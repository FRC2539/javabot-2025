package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmrollerIO.ArmrollerIOInputs;
import org.littletonrobotics.junction.Logger;

public class ArmrollerSubsystem extends SubsystemBase {

    private ArmrollerIO piviotIO;

    private ArmrollerIOInputs armrollerInputs = new ArmrollerIOInputs();

    public ArmrollerSubsystem(ArmrollerIO armrollerIO) {
        this.piviotIO = armrollerIO;
    }

    public void periodic() {

        Logger.recordOutput("Armroller/Voltage", armrollerInputs.voltage);
    }

    public Command intakeSpin() {
        return setVoltage(12);
    }

    public Command edjectSpin() {
        return setVoltage(-12);
    }

    public Command setVoltage(double voltage) {
        return run(
                () -> {
                    piviotIO.setVoltage(voltage);
                });
    }
}
