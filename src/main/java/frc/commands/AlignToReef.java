package frc.commands;

import static frc.robot.constants.VisionConstants.aprilTagLayout;

import java.security.spec.XECPublicKeySpec;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

public class AlignToReef extends Command {
    
    CommandSwerveDrivetrain drivetrain;
    Vision vision;
    int cameraId;
    DoubleSupplier xSupplier;


    PIDController yController = new PIDController(3, 0, 0);
    Pose2d targetPose; 

    public AlignToReef(CommandSwerveDrivetrain drivetrain, Vision vision, int cameraId, DoubleSupplier xSupplier) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.cameraId = cameraId;
        this.xSupplier = xSupplier;

        addRequirements(drivetrain, vision);
    }
    @Override
    public void initialize() {
        targetPose = aprilTagLayout.getTagPose(16).get().toPose2d();
     
        yController.setSetpoint(0);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getRobotPose();
        Transform2d offset = robotPose.minus(targetPose);
        double yVelocity = yController.calculate(offset.getY());
        drivetrain.setControl(
                drivetrain.driveRobotRelative(
                        new ChassisSpeeds(-yVelocity, xSupplier.getAsDouble(), 0)));
    }

    @Override
    public boolean isFinished() {
        return yController.atSetpoint();
    }
}
