package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ClimberConstants;
import frc.robot.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
    // TBD: Hardcode IDs or add support to make changeable in method
    private final TalonFX climbermotor =
            new TalonFX(ClimberConstants.CLIMBER_ID, ClimberConstants.CANbus);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    // Status signals for efficient updates
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<AngularVelocity> speedSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Current> currentSignal;

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

        // Initialize status signals
        positionSignal = climbermotor.getPosition();
        voltageSignal = climbermotor.getMotorVoltage();
        speedSignal = climbermotor.getVelocity();
        temperatureSignal = climbermotor.getDeviceTemp();
        currentSignal = climbermotor.getStatorCurrent();

        // Set update frequency and optimize bus utilization
        PhoenixUtil.tryUntilOk(
                5,
                () ->
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                positionSignal,
                                voltageSignal,
                                speedSignal,
                                temperatureSignal,
                                currentSignal));

        PhoenixUtil.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(0, climbermotor));
    }

    public void updateInputs(ClimberIOInputs inputs) {
        // Refresh all signals at once
        BaseStatusSignal.refreshAll(
                positionSignal, voltageSignal, speedSignal, temperatureSignal, currentSignal);

        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.speed = speedSignal.getValueAsDouble();
        inputs.temperature = temperatureSignal.getValueAsDouble();
        inputs.current = currentSignal.getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        climbermotor.setVoltage(voltage);
    }

    public void setPosition(double position) {
        if (position > ClimberConstants.upperLimit) {
            position = ClimberConstants.upperLimit;
        }
        if (position < ClimberConstants.lowerLimit) {
            position = ClimberConstants.lowerLimit;
        }

        climbermotor.setControl(motionMagicVoltage.withPosition(position));
    }

    public void resetPosition(double position) {
        climbermotor.setPosition(position);
    }
}
