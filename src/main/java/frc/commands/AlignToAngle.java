package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class AlignToAngle extends Command {
    public CommandSwerveDrivetrain drivetrain;
    private Rotation2d target;
    public PIDController controller = new PIDController(4, 0, 0);
    private boolean isFieldRelative;
    private DoubleSupplier xVelocity = () -> 0;
    private DoubleSupplier yVelocity = () -> 0;

    public AlignToAngle(
            CommandSwerveDrivetrain drivetrain, Rotation2d target, boolean fieldRelative) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.isFieldRelative = fieldRelative;
        addRequirements(drivetrain);
    }

    public AlignToAngle(
            CommandSwerveDrivetrain drivetrain,
            Rotation2d target,
            boolean fieldRelative,
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.isFieldRelative = fieldRelative;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(
                Rotation2d.fromDegrees(0.5).getRadians(), Rotation2d.fromDegrees(1).getRadians());

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
                drivetrain.driveFieldRelative(
                        new ChassisSpeeds(
                                xVelocity.getAsDouble(), yVelocity.getAsDouble(), thetaVelocity)));
        System.out.println(thetaVelocity);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
