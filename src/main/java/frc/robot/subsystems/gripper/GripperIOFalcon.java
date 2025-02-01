package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.GripperConstants;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX armRoller = new TalonFX(GripperConstants.id, GripperConstants.canbus);

    public GripperIOFalcon() {
        armRoller.setPosition(0);

        armRoller
                .getConfigurator()
                .apply(new TalonFXConfiguration().withCurrentLimits(GripperConstants.currentLimit));

        armRoller.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(GripperIOInputs inputs) {
        inputs.speed = armRoller.getVelocity().getValueAsDouble();
        inputs.voltage = armRoller.getMotorVoltage().getValueAsDouble();
        inputs.current = armRoller.getStatorCurrent().getValueAsDouble();
        inputs.temperature = armRoller.getDeviceTemp().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        armRoller.setVoltage(voltage);
    }
}
