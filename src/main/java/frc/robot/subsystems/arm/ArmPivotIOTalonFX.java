package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;
import frc.robot.util.PhoenixUtil;

public class ArmPivotIOTalonFX implements ArmPivotIO {
    private final TalonFX armPivotMotor =
            new TalonFX(
                    ArmConstants.ARM_PIVOT_MOTOR_ID,
                    ArmConstants.ARM_PIVOT_CANBUS); // not the correct ID

    // private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final DutyCycleEncoder throughboreEncoder =
            new DutyCycleEncoder(ArmConstants.ARM_THROUGHBORE_ENCODER_ID, 2 * Math.PI, 0);

    private double lastVoltage = 0;

    // Status signals for efficient updates
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Current> currentSignal;

    public ArmPivotIOTalonFX() {
        armPivotMotor.setPosition(0);

        TalonFXConfigurator talonConfig = armPivotMotor.getConfigurator();
        talonConfig.apply(
                new TalonFXConfiguration().withCurrentLimits(ArmConstants.currentLimitConfigs));
        armPivotMotor.setNeutralMode(NeutralModeValue.Brake);

        // Initialize status signals
        positionSignal = armPivotMotor.getPosition();
        voltageSignal = armPivotMotor.getMotorVoltage();
        velocitySignal = armPivotMotor.getVelocity();
        temperatureSignal = armPivotMotor.getDeviceTemp();
        currentSignal = armPivotMotor.getStatorCurrent();

        // Set update frequency and optimize bus utilization
        PhoenixUtil.tryUntilOk(5, () -> 
            BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
                positionSignal, voltageSignal, velocitySignal, temperatureSignal, currentSignal));
        
        PhoenixUtil.tryUntilOk(5, () -> 
            ParentDevice.optimizeBusUtilizationForAll(0, armPivotMotor));
    }

    public void updateInputs(ArmPivotIOInputs inputs) {
        // Refresh all signals at once
        BaseStatusSignal.refreshAll(
            positionSignal, voltageSignal, velocitySignal, temperatureSignal, currentSignal);

        inputs.position = positionSignal.refresh().getValueAsDouble();
        inputs.voltage = voltageSignal.refresh().getValueAsDouble();
        inputs.velocity = velocitySignal.refresh().getValueAsDouble();
        inputs.temperature = temperatureSignal.refresh().getValueAsDouble();
        inputs.current = currentSignal.refresh().getValueAsDouble();
        inputs.throughboreEncoderPosition = throughboreEncoder.get();
        inputs.throughboreConnected = throughboreEncoder.isConnected();

        if (inputs.throughboreEncoderPosition >= ArmConstants.upperLimit && lastVoltage > 0) {
            lastVoltage = 0;
        }
        if (inputs.throughboreEncoderPosition <= ArmConstants.lowerLimit && lastVoltage < 0) {
            lastVoltage = 0;
        }
        if (!inputs.throughboreConnected) {
            lastVoltage = 0;
        }
        armPivotMotor.setVoltage(lastVoltage);
    }

    public void setVoltage(double voltage) {
        lastVoltage = voltage;
    }
}
