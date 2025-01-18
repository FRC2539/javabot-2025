package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakerollerIO.IntakerollerIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakerollerSubsystems extends SubsystemBase {
    private IntakerollerIO piviotIO;
    private IntakerollerIOInputs intakeInputs = new IntakerollerIOInputs();

    public IntakerollerSubsystems(
            IntakerollerIOSim intakerollerIO, IntakerollerTalonFX intakerollerTalonFX) {
        piviotIO = intakerollerIO;
    }

    public void periodic() {
        piviotIO.updateInputs(intakeInputs);

        Logger.recordOutput("Intakeroller/Voltage", intakeInputs.voltage);
    }

    public Command intake() {
        return run(() -> piviotIO.setVoltage(12));
    }

    public Command eject() {
        return run(() -> piviotIO.setVoltage(-12));
    }
}
