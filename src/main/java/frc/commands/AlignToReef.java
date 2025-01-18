package frc.commands;

import static frc.robot.constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class AlignToReef extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private DoubleSupplier xVelocity;
    private DoubleSupplier yVelocity;

    private PIDController thetaController = new PIDController(4, 0, 0);
    private PIDController yController = new PIDController(6, 0, 0);
    private int tagId;
    private Pose2d targetPose;
    private double offset;

    public AlignToReef(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            double alignmentOffset,
            int tagId) {
        this.drivetrain = drivetrain;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.tagId = tagId;
        this.offset = alignmentOffset;
    }

    @Override
    public void initialize() {

        // Camera id
        // tagId
        // Rotation to face the tag

        targetPose = aprilTagLayout.getTagPose(tagId).get().toPose2d();

        thetaController.setSetpoint(Math.PI);
        yController.setSetpoint(offset);
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(0.5));
        yController.setTolerance(Units.inchesToMeters(0.4));
    }

    @Override
    public void execute() {
        // double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
        Pose2d currentPose = drivetrain.getState().Pose;

        Transform2d offset = currentPose.minus(targetPose);

        double thetaVelocity = thetaController.calculate(offset.getRotation().getRadians());
        if (thetaController.atSetpoint()) {
            thetaVelocity = 0;
        }
        double yVelocityController = yController.calculate(offset.getY());
        if (yController.atSetpoint()) {
            yVelocityController = 0;
        }

        Rotation2d tagRotation = targetPose.getRotation();

        ChassisSpeeds driverCommandedVelocities =
                new ChassisSpeeds(xVelocity.getAsDouble(), yVelocity.getAsDouble(), 0);

        ChassisSpeeds fieldCommandedVelocities =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        driverCommandedVelocities, drivetrain.getOperatorForwardDirection());

        ChassisSpeeds tagRelativeCommandedVelocities =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldCommandedVelocities, tagRotation);

        tagRelativeCommandedVelocities.vyMetersPerSecond = yVelocityController;
        tagRelativeCommandedVelocities.omegaRadiansPerSecond = thetaVelocity;

        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(tagRelativeCommandedVelocities, tagRotation);

        // System.out.println(offset.getRotation().getRadians());
        drivetrain.setControl(drivetrain.driveFieldRelative(fieldRelativeSpeeds));
    }

    @Override
    public boolean isFinished() {
        return false;
        // return thetaController.atSetpoint() && yController.atSetpoint();
    }
}
