package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {
    // TBD: Hardcode IDs or add support to make changeable in method
    private final TalonFX climbermotor = new TalonFX(ClimberConstants.id, ClimberConstants.CANbus);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ClimberIOTalonFX() {
        climbermotor.setPosition(0);

        motionMagicVoltage.Slot = 0;

        TalonFXConfigurator talonConfig = climbermotor.getConfigurator();

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ClimberConstants.upperLimit)
                        .withReverseSoftLimitThreshold(ClimberConstants.lowerLimit)
                        .withReverseSoftLimitEnable(true);

        talonConfig.apply(
                new TalonFXConfiguration()
                        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                        .withSlot0(ClimberConstants.slot0Configs)
                        .withMotionMagic(ClimberConstants.motionMagicConfigs)
                        .withCurrentLimits(ClimberConstants.currentLimitsConfigs));

        climbermotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.position = climbermotor.getPosition().getValueAsDouble();
        inputs.voltage = climbermotor.getMotorVoltage().getValueAsDouble();
        inputs.speed = climbermotor.getVelocity().getValueAsDouble();
        inputs.temperature = climbermotor.getDeviceTemp().getValueAsDouble();
        inputs.current = climbermotor.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        climbermotor.setVoltage(voltage);
    }

    public void setPosition(double position) {
        climbermotor.setControl(motionMagicVoltage.withPosition(position));
    }

    public void resetPosition(double position){
        climbermotor.setPosition(position);
    }
}
