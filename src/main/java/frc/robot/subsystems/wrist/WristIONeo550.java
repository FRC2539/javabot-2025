package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.WristConstants;

public class WristIONeo550 implements WristIO {
    private SparkMax wristMotor =
            new SparkMax(WristConstants.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder throughboreEncoder =
            new DutyCycleEncoder(WristConstants.WRIST_THROUGHBORE_ENCODER_ID, 2 * Math.PI, 0);

    public WristIONeo550() {
        wristMotor.getEncoder().setPosition(0);

        SparkBaseConfig config =
                new SparkMaxConfig()
                        .smartCurrentLimit((int) WristConstants.WristCurrent)
                        .secondaryCurrentLimit(WristConstants.WristCurrent)
                        .idleMode(IdleMode.kBrake);
        wristMotor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean shutdown = false;

    public void updateInputs(WristIOInputs inputs) {
        inputs.position = wristMotor.getEncoder().getPosition();
        inputs.voltage = wristMotor.getBusVoltage() * wristMotor.getAppliedOutput();
        inputs.current = wristMotor.getOutputCurrent();
        inputs.temperature = wristMotor.getMotorTemperature();
        inputs.throughboreEncoderPosition = throughboreEncoder.get();
        inputs.throughboreConnected = throughboreEncoder.isConnected();

        if (inputs.temperature > 60) {
            shutdown = true;
            wristMotor.stopMotor();
        } else if (inputs.temperature < 58) {
            shutdown = false;
        }

        inputs.shutdown = shutdown;
    }

    public void setVoltage(double voltage) {
        if (shutdown) return;
        wristMotor.setVoltage(voltage);
    }
}
