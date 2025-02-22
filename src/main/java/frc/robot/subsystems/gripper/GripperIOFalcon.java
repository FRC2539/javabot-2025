package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.GripperConstants;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX armRoller = new TalonFX(GripperConstants.id, GripperConstants.canbus);

    private AnalogInput gripperGPSensor = new AnalogInput(GripperConstants.gripperSensorChannel);

    public GripperIOFalcon() {
        armRoller.setPosition(0);

        armRoller
                .getConfigurator()
                .apply(
                        new TalonFXConfiguration()
                                .withCurrentLimits(GripperConstants.currentLimit)
                                .withMotorOutput(
                                        new MotorOutputConfigs()
                                                .withInverted(InvertedValue.Clockwise_Positive)));
        armRoller.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(GripperIOInputs inputs) {
        inputs.speed = armRoller.getVelocity().getValueAsDouble();
        inputs.voltage = armRoller.getMotorVoltage().getValueAsDouble();
        inputs.current = armRoller.getStatorCurrent().getValueAsDouble();
        inputs.temperature = armRoller.getDeviceTemp().getValueAsDouble();
        inputs.sensor = gripperGPSensor.getValue() < 50;
    }

    public void setVoltage(double voltage) {
        armRoller.setVoltage(voltage);
    }
}
