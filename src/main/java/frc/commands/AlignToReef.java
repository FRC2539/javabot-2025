package frc.commands;

import static frc.robot.constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class alignToReef extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private DoubleSupplier xVelocity;
    

    private PIDController thetaController = new PIDController(4, 0, 0);
    private PIDController yController = new PIDController(6, 0, 0);
    private int cameraId;
    private int tagId;
    private Pose2d targetPose;

    public alignToReef(CommandSwerveDrivetrain drivetrain, DoubleSupplier xVelocity) {
        this.drivetrain = drivetrain;
        this.xVelocity = xVelocity;
    }
    @Override
    public void initialize() {

        // Camera id
        // tagId
        // Rotation to face the tag
        int cameraId = 0;
        int tagId = 10;
        
        targetPose = aprilTagLayout.getTagPose(tagId).get().toPose2d();

        thetaController.setSetpoint(0);
        yController.setSetpoint(0);
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(0.5));
        yController.setTolerance(Units.inchesToMeters(1), Units.inchesToMeters(1));
    }

    @Override
    public void execute() {
        double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
        Pose2d currentPose = drivetrain.getState().Pose;


        Transform2d offset = currentPose.minus(targetPose);

        double thetaVelocity = thetaController.calculate(currentHeading);
        double yVelocity = yController.calculate(offset.getY());

        System.out.println(currentHeading);
        drivetrain.setControl(
                drivetrain.driveRobotRelative(
                        new ChassisSpeeds(xVelocity.getAsDouble(), -yVelocity, thetaVelocity)));

        
        
    }

    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint() && yController.atSetpoint();
    }

    
}
