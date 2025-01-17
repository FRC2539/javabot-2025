package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmrollerFalcon implements ArmrollerIO {
    private final TalonFX armRoller = new TalonFX(51);

    public ArmrollerFalcon() {
        armRoller.setPosition(0);
    }

    public void updateInputs(ArmrollerIOInputs inputs) {
        inputs.speed = armRoller.getVelocity().refresh().getValueAsDouble();
        inputs.voltage = armRoller.getMotorVoltage().refresh().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        armRoller.setVoltage(voltage);
    }

    public void updateInputs(ArmrollerIO inputs) {}
}
