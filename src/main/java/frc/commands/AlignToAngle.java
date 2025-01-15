package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToAngle extends Command {
    public CommandSwerveDrivetrain drivetrain;
    private Rotation2d target;
    public PIDController controller = new PIDController(6, 0, 0);
    private boolean isFieldRelative;

    public AlignToAngle(
            CommandSwerveDrivetrain drivetrain, Rotation2d target, boolean fieldRelative) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.isFieldRelative = fieldRelative;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(0.03, 0.0);

        if (isFieldRelative) {
            controller.setSetpoint(target.getRadians());
        } else {
            Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
            controller.setSetpoint(currentHeading.plus(target).getRadians());
        }
    }

    @Override
    public void execute() {

        double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();

        double thetaVelocity = controller.calculate(currentHeading);
        drivetrain.setControl(
                drivetrain.driveFieldRelative(new ChassisSpeeds(0, 0, thetaVelocity)));
        System.out.println(thetaVelocity);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
