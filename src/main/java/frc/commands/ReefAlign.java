package frc.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.constants.AligningConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class ReefAlign extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private DoubleSupplier txSupplier;
    private double txOffset;
    private PIDController xController =
            new PIDController(AligningConstants.KP, AligningConstants.KI, AligningConstants.KD);
    private ThrustmasterJoystick driveJoystick;

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
        xController.setTolerance(AligningConstants.aligningDeadband);
        xController.setSetpoint(txOffset);
    }

    @Override
    public void execute() {
        double xVelocity = xController.calculate(txSupplier.getAsDouble());
        double yVelocity =
                sps(deadband(driveJoystick.getXAxis().get(), 0.1))
                        * GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond);

        drivetrain.setControl(
                drivetrain.driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, 0)));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint();
    }

    @Override
    public void end(boolean isInterrupted) {}

    private double deadband(double value, double deadband) {
        if (value <= deadband && -deadband <= value) {
            return 0;
        }

        return value;
    }

    private double sps(double value) {
        return value * Math.abs(value);
    }
}
