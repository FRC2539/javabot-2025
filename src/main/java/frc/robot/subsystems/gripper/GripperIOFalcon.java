package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GripperConstants;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX armRoller = new TalonFX(GripperConstants.id);

    public GripperIOFalcon() {
        armRoller.setPosition(0);
    }

    public void updateInputs(GripperIOInputs inputs) {
        inputs.speed = armRoller.getVelocity().refresh().getValueAsDouble();
        inputs.voltage = armRoller.getMotorVoltage().refresh().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        armRoller.setVoltage(voltage);
    }
}
