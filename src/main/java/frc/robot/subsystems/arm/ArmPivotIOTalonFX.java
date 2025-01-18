package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ArmConstants;

public class ArmPivotIOTalonFX implements ArmPivotIO {
    private final TalonFX armPivotMotor =
            new TalonFX(ArmConstants.ARM_PIVOT_MOTOR_ID, "CANivore"); // not the correct ID

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ArmPivotIOTalonFX() {
        armPivotMotor.setPosition(0);

        motionMagicVoltage.Slot = 0;

        TalonFXConfigurator talonConfig = armPivotMotor.getConfigurator();

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();

        talonConfig.apply(
                new TalonFXConfiguration()
                        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                        .withSlot0(ArmConstants.slot0Configs)
                        .withMotionMagic(ArmConstants.motionMagicConfigs));
    }

    public void updateInputs(ArmPivotIOInputs inputs) {

        inputs.position = armPivotMotor.getPosition().refresh().getValueAsDouble();
        inputs.voltage = armPivotMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = armPivotMotor.getVelocity().refresh().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        armPivotMotor.setVoltage(voltage);
    }

    public void setPosition(double position) {
        armPivotMotor.setControl(motionMagicVoltage.withPosition(position));
    }
}
