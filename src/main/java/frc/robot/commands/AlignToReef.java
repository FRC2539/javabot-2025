package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AligningConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class AlignToReef extends Command {
    private CommandSwerveDrivetrain driveSubsystem;
    private DoubleSupplier txSupplier;

    private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, null);
    private double goalTx;

    public AlignToReef(
            CommandSwerveDrivetrain driveSubsystem,
            DoubleSupplier txSupplier,
            boolean alignR) {
        this.driveSubsystem = driveSubsystem;
        this.txSupplier = txSupplier;

        this.goalTx = alignR ? AligningConstants.targetTxR : AligningConstants.targetTxL;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        xController.setGoal(goalTx);
        xController.setTolerance(AligningConstants.angleDeadband);
        
    }

    @Override
    public void execute() {
        double xVelocity = xController.calculate(txSupplier.getAsDouble());
        driveSubsystem.driveRobotRelative(new ChassisSpeeds(xVelocity, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal();
    }
}
