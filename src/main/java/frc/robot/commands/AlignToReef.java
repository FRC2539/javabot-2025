package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class AlignToReef extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final PIDController xController = new PIDController(0.0001, 0, 0);
    private DoubleSupplier txSupplier;

    public AlignToReef(CommandSwerveDrivetrain commandSwerveDrivetrain, DoubleSupplier txSupplier) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.txSupplier = txSupplier;

        xController.setTolerance(2);

        addRequirements(commandSwerveDrivetrain);
    }

    @Override
    public void execute() {
        double xVelocity = xController.calculate(txSupplier.getAsDouble());
        System.out.println(xVelocity + "   " + txSupplier.getAsDouble() );
        commandSwerveDrivetrain.applyRequest(() -> commandSwerveDrivetrain.driveRobotRelative(xVelocity, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint();
    }
}
