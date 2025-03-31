package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignToReef extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private DoubleSupplier xVelocity;
    private DoubleSupplier yVelocity;

    private PIDController thetaController = new PIDController(4, 0, 0);
    //private ProfiledPIDController yController = new ProfiledPIDController(8, 0, 0, new TrapezoidProfile.Constraints(5.5, 8));
    private PIDController yController = new PIDController(8, 0, 0);

    private Pose2d targetPose;
    private double offset;
    private Rotation2d rotationOffset;

    private Timer slewerTimer = new Timer();

    public AlignToReef(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            double alignmentOffset,
            Pose2d alignmentPose,
            Rotation2d rotationOffset) {
        this.drivetrain = drivetrain;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.offset = alignmentOffset;
        this.targetPose = alignmentPose;
        this.rotationOffset = rotationOffset;
    }

    @Override
    public void initialize() {

        // Camera id
        // tagId
        // Rotation to face the tag
        slewerTimer.restart();
        Pose2d tCurrentPose = drivetrain.getState().Pose;
        Logger.recordOutput("/AutoAlign/CurrentPose", tCurrentPose);
        Logger.recordOutput("/AutoAlign/TargetPose", targetPose);
        Logger.recordOutput("/AutoAlign/Offset", offset);

        thetaController.setSetpoint(rotationOffset.getRadians());
        yController.setSetpoint(offset);
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(0.5));
        yController.setTolerance(Units.inchesToMeters(0.2));
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
