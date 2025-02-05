package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;

public class ArmPivotIOTalonFX implements ArmPivotIO {
    private final TalonFX armPivotMotor =
            new TalonFX(
                    ArmConstants.ARM_PIVOT_MOTOR_ID,
                    ArmConstants.ARM_PIVOT_CANBUS); // not the correct ID

    // private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final DutyCycleEncoder throughboreEncoder =
            new DutyCycleEncoder(ArmConstants.ARM_THROUGHBORE_ENCODER_ID, 2 * Math.PI, 0);

    public ArmPivotIOTalonFX() {
        armPivotMotor.setPosition(0);

        // motionMagicVoltage.Slot = 0;

        TalonFXConfigurator talonConfig = armPivotMotor.getConfigurator();

        // SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();

        talonConfig.apply(
                new TalonFXConfiguration().withCurrentLimits(ArmConstants.currentLimitConfigs));

        armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(ArmPivotIOInputs inputs) {

        inputs.position = armPivotMotor.getPosition().getValueAsDouble();
        inputs.voltage = armPivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.velocity = armPivotMotor.getVelocity().getValueAsDouble();
        inputs.temperature = armPivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.current = armPivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.throughboreEncoderPosition = throughboreEncoder.get();
        inputs.throughboreConnected = throughboreEncoder.isConnected();
    }

    public void setVoltage(double voltage) {
        armPivotMotor.setVoltage(voltage);
    }
}
