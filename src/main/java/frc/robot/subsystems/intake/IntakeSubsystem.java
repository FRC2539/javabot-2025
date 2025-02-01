package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeRollerIO piviotIO;
    private IntakeRollerIOInputsAutoLogged intakeInputs = new IntakeRollerIOInputsAutoLogged();

    private FlipperIO flipperIO;
    private FlipperIOInputsAutoLogged flipperInputs = new FlipperIOInputsAutoLogged();

    LoggedNetworkNumber flippervoltage = new LoggedNetworkNumber("Flipper Voltage", 0);
    LoggedNetworkNumber rollervoltage = new LoggedNetworkNumber("Intake Voltage", 0);

    public IntakeSubsystem(IntakeRollerIO intakerollerIO, FlipperIO sflipperIO) {
        piviotIO = intakerollerIO;
        flipperIO = sflipperIO;
        setDefaultCommand(
                run(
                        () -> {
                            piviotIO.setVoltage(0);
                            flipperIO.setClose();
                        }));
    }

    public void periodic() {
        piviotIO.updateInputs(intakeInputs);
        flipperIO.updateInputs(flipperInputs);

        Logger.processInputs("RealOutputs/Flipper", flipperInputs);
        Logger.processInputs("RealOutputs/IntakeRoller", intakeInputs);

        Logger.recordOutput("Flipper/Position", flipperInputs.position);
    }

    public Command flipperTuneable() {
        return run(
                () -> {
                    double voltage = flippervoltage.get();
                    flipperIO.setVoltage(voltage);
                });
    }

    public Command rollerTuneable() {
        return run(
                () -> {
                    double voltage = rollervoltage.get();
                    piviotIO.setVoltage(voltage);
                });
    }

    public Command zeroflipper() {
        return runOnce(
                () -> {
                    flipperIO.resetPosition(0);
                });
    }

    public Command intake() {
        return run(() -> piviotIO.setVoltage(12));
    }

    public Command eject() {
        return run(() -> piviotIO.setVoltage(-12));
    }

    public Command openIntake() {
        return run(() -> flipperIO.setOpen());
    }

    public Command openAndRun() {
        return run(
                () -> {
                    flipperIO.setOpen();
                    piviotIO.setVoltage(12);
                });
    }

    public Command openAndEject() {
        return run(
                () -> {
                    flipperIO.setOpen();
                    piviotIO.setVoltage(-12);
                });
    }

    public Command closeIntake() {
        return run(() -> flipperIO.setClose());
    }
}
