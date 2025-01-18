package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.FlipperIO.FlipperIOInputs;
import frc.robot.subsystems.intake.IntakeRollerIO.IntakeRollerIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeRollerIO piviotIO;
    private IntakeRollerIOInputs intakeInputs = new IntakeRollerIOInputs();

    private FlipperIO flipperIO;
    private FlipperIOInputs flipperInputs = new FlipperIOInputs();

    private final double lowerLimit = 0.0;
    private final double upperLimit = 100.0;

    public IntakeSubsystem(IntakeRollerIO intakerollerIO, FlipperIO flipperIO) {
        piviotIO = intakerollerIO;
    }

    public void periodic() {
        piviotIO.updateInputs(intakeInputs);

        Logger.recordOutput("IntakeRoller/Voltage", intakeInputs.voltage);
        Logger.recordOutput("FlipperFlip/Voltage", flipperInputs.voltage);

        Logger.recordOutput("Flipper/Position", flipperInputs.position);
        if (flipperInputs.voltage < 0 && flipperInputs.position <= lowerLimit) {
            this.piviotIO.setVoltage(0);
        }
        if (flipperInputs.voltage > 0 && flipperInputs.position >= upperLimit) {
            this.piviotIO.setVoltage(0);
        }

        setDefaultCommand(run(() -> piviotIO.setVoltage(0)));
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
