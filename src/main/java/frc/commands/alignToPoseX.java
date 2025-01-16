package frc.commands;

import static frc.robot.constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class alignToPoseX extends Command {
    private Vision visionSubsystem;
    private CommandSwerveDrivetrain drivetrainSubsystem;
    private int tagId;
    private int cameraId;
    Pose3d tagPose;
    Pose3d currentPose;
    PIDController xController = new PIDController(0, 0, 0);
    DoubleSupplier yVelocity;

    public alignToPoseX(
            CommandSwerveDrivetrain drivetrainSubsystem,
            Vision visionSubsystem,
            int tagId,
            int cameraId,
            DoubleSupplier yVelocity) {
        this.visionSubsystem = visionSubsystem;
        this.tagId = tagId;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.cameraId = cameraId;
        this.yVelocity = yVelocity;
    }

    @Override
    public void initialize() {
        tagPose = aprilTagLayout.getTagPose(tagId).get();
        xController.setSetpoint(0);
    }

    @Override
    public void execute() {
        currentPose = visionSubsystem.getNewestPoseObservation(cameraId).get().pose();
        Transform3d offset = currentPose.minus(tagPose);
        xController.calculate(offset.getX());
        drivetrainSubsystem.setControl(
                drivetrainSubsystem.driveRobotRelative(
                        new ChassisSpeeds(yVelocity.getAsDouble(), yVelocity.getAsDouble(), 0)));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint();
    }
}
