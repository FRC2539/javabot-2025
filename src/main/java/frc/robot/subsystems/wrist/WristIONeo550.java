package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkBase.ControlType;
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
        wristMotor.getEncoder().setPosition(-Math.PI / 2 / Math.PI * 76.1);

        SparkBaseConfig config =
                new SparkMaxConfig()
                        .smartCurrentLimit((int) WristConstants.WristCurrent)
                        .secondaryCurrentLimit(WristConstants.WristCurrent)
                        .idleMode(IdleMode.kBrake);
        wristMotor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean shutdown = false;

    private double lastVoltage = 0;

    private boolean usingVoltage = true;

    public void updateInputs(WristIOInputs inputs) {
        inputs.voltage = wristMotor.getBusVoltage() * wristMotor.getAppliedOutput();
        inputs.current = wristMotor.getOutputCurrent();
        inputs.temperature = wristMotor.getMotorTemperature();
        inputs.throughboreEncoderPosition = throughboreEncoder.get() - 4.22;
        wristMotor.getEncoder().setPosition(inputs.throughboreEncoderPosition  / Math.PI * 76.1);
        inputs.position = wristMotor.getEncoder().getPosition();
        inputs.throughboreConnected = throughboreEncoder.isConnected();

        if (inputs.temperature > 60) {
            shutdown = true;
        } else if (inputs.temperature < 58) {
            shutdown = false;
        }

        inputs.shutdown = shutdown;

        if (inputs.throughboreEncoderPosition >= WristConstants.upperLimit && (inputs.voltage > 0 || lastVoltage > 0)) {
            lastVoltage = 0;
            usingVoltage = true;
        }
        if (inputs.throughboreEncoderPosition <= WristConstants.lowerLimit && (inputs.voltage < 0 || lastVoltage < 0)) {
            lastVoltage = 0;
            usingVoltage = true;
        }
        if (!inputs.throughboreConnected) {
            lastVoltage = 0;
            usingVoltage = true;
        }
        if (shutdown) {
            lastVoltage = 0;
            usingVoltage = true;
        }

        if (usingVoltage) {
            wristMotor.setVoltage(lastVoltage);
        }
    }

    public void setVoltage(double voltage) {
        lastVoltage = voltage;
        usingVoltage = true;
    }

    public void setPositionControl(double reference) {
        wristMotor.getClosedLoopController().setReference(reference / Math.PI * 76.1, ControlType.kPosition);
        usingVoltage = false;
    }
}
