package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.constants.AligningConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class ReefAlign extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private DoubleSupplier txSupplier;
    private double txOffset;
    private PIDController xController =
            new PIDController(AligningConstants.Kp, AligningConstants.Ki, AligningConstants.Kd);
    private ThrustmasterJoystick driveJoystick;

    // private SwerveRequest.FieldCentricFacingAngle rotateToAngle = new
    // SwerveRequest.FieldCentricFacingAngle();

    public ReefAlign(
            CommandSwerveDrivetrain drivetrain,
            ThrustmasterJoystick driveJoystick,
            DoubleSupplier txSupplier,
            double txOffset) {
        this.drivetrain = drivetrain;
        this.txSupplier = txSupplier;
        this.txOffset = txOffset;
        this.driveJoystick = driveJoystick;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.setTolerance(AligningConstants.aligningDeadband, 0.5);
        xController.setSetpoint(txOffset);

        // rotateToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // rotateToAngle.HeadingController.setPID(0.8, 0.0025, 0.0);
        // rotateToAngle.HeadingController.setTolerance(0.05);
        // rotateToAngle.withTargetDirection(new Rotation2d(0));

    }

    @Override
    public void execute() {
        double xVelocity = xController.calculate(txSupplier.getAsDouble());
        // drivetrain.setControl(drivetrain.driveRobotRelative(xVelocity, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint();
    }

    @Override
    public void end(boolean isInterrupted) {}
}
