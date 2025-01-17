package frc.commands;

import static frc.robot.constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
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
    Pose2d tagPose;
    Pose2d currentPose;
    PIDController yController = new PIDController(0.5, 0, 0);
    DoubleSupplier xVelocity;

    public alignToPoseX(
            CommandSwerveDrivetrain drivetrainSubsystem,
            Vision visionSubsystem,
            int tagId,
            int cameraId,
            DoubleSupplier xVelocity) {
        this.visionSubsystem = visionSubsystem;
        this.tagId = tagId;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.cameraId = cameraId;
        this.xVelocity = xVelocity;
    }

    @Override
    public void initialize() {
        tagPose = aprilTagLayout.getTagPose(tagId).get().toPose2d();
        yController.setSetpoint(0);
    }

    @Override
    public void execute() {
        currentPose = drivetrainSubsystem.getState().Pose;
        Transform2d offset = currentPose.minus(tagPose);
        double yVelocity = yController.calculate(offset.getY());
        drivetrainSubsystem.setControl(
                drivetrainSubsystem.driveRobotRelative(
                        new ChassisSpeeds(xVelocity.getAsDouble(), -yVelocity, 0)));
    }

    @Override
    public boolean isFinished() {
        return yController.atSetpoint();
    }
}
