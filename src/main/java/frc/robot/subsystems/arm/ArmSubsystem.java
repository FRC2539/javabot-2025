package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private ArmIO armIO;
    private ArmIO.ArmIOInputs armInputs = new ArmIO.ArmIOInputs();

    public ArmSubsystem(ArmIO armIO) {
        this.armIO = armIO;
    }

    public void periodic() {
        // periodic stuff i guess
    }

    public Command moveArmUp() {
        return run(
                () -> {
                    armIO.setArmVoltage(12);
                });
    }

    public Command moveArmDown() {
        return run(
                () -> {
                    armIO.setArmVoltage(-12);
                });
    }

    public Command moveWristUp() {
        return run(
                () -> {
                    armIO.setWristVoltage(12);
                });
    }

    public Command moveWristDown() {
        return run(
                () -> {
                    armIO.setWristVoltage(-12);
                });
    }
}
