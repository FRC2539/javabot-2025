package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeRollerIO piviotIO;
    private IntakeRollerIOInputsAutoLogged intakeInputs = new IntakeRollerIOInputsAutoLogged();

    private FlipperIO flipperIO;
    private FlipperIOInputsAutoLogged flipperInputs = new FlipperIOInputsAutoLogged();

    private final double lowerLimit = 0.0;
    private final double upperLimit = 100.0;

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
        if (flipperInputs.voltage < 0 && flipperInputs.position <= lowerLimit) {
            this.piviotIO.setVoltage(0);
        }
        if (flipperInputs.voltage > 0 && flipperInputs.position >= upperLimit) {
            this.piviotIO.setVoltage(0);
        }
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

    public Command closeIntake() {
        return run(() -> flipperIO.setClose());
    }
}
