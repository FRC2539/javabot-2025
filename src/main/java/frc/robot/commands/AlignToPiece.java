package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.function.Supplier;

public class AlignToPiece extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private Supplier<ChassisSpeeds> velocity;

    private PIDController yController = new PIDController(6, 0, 0);
    private Supplier<Pose2d> targetPose;
    private double offset;
    private Rotation2d rotationOffset;

    /**
     * Creates a command that aligns to a piece horizontally. The driver maintains control over
     * rotation and the perpendicular direction.
     *
     * @param drivetrain
     * @param xVelocity
     * @param yVelocity
     * @param alignmentOffset
     * @param piecePoseSupplier
     * @param rotationOffset
     */
    public AlignToPiece(
            CommandSwerveDrivetrain drivetrain,
            Supplier<ChassisSpeeds> velocity,
            double alignmentOffset,
            Supplier<Pose2d> piecePoseSupplier,
            Rotation2d rotationOffset) {
        this.drivetrain = drivetrain;
        this.velocity = velocity;
        this.offset = alignmentOffset;
        this.targetPose = piecePoseSupplier;
        this.rotationOffset = rotationOffset.unaryMinus();
    }

    @Override
    public void initialize() {

        // Camera id
        // tagId
        // Rotation to face the tag
        yController.setSetpoint(offset);
        yController.setTolerance(Units.inchesToMeters(0.4));
    }

    @Override
    public void execute() {
        // double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
        Pose2d currentPose = drivetrain.getState().Pose;

        Pose2d piecePose = targetPose.get();

        piecePose =
                new Pose2d(
                        piecePose.getTranslation(), currentPose.getRotation().plus(rotationOffset));

        Transform2d offset = currentPose.minus(piecePose);

        ChassisSpeeds driverCommandedVelocities = velocity.get();

        double thetaVelocity = driverCommandedVelocities.omegaRadiansPerSecond;

        double yVelocityController = yController.calculate(offset.getY());
        if (yController.atSetpoint()) {
            yVelocityController = 0;
        }

        Rotation2d tagRotation = piecePose.getRotation();

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
