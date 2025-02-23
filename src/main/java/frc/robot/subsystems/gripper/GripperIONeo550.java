package frc.robot.subsystems.gripper;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.WristConstants;

public class GripperIONeo550 implements GripperIO {
    private SparkMax leftSparkMax =
            new SparkMax(GripperConstants.id, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightSparkMax =
            new SparkMax(GripperConstants.id, SparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder throughboreEncoder =
            new DutyCycleEncoder(WristConstants.WRIST_THROUGHBORE_ENCODER_ID, 2 * Math.PI, 0);

    public GripperIONeo550() {
        SparkBaseConfig config =
                new SparkMaxConfig()
                        .smartCurrentLimit((int) WristConstants.WristCurrent)
                        .secondaryCurrentLimit(WristConstants.WristCurrent)
                        .idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(Math.PI / 76.1);

        config.closedLoop
                .p(WristConstants.WRIST_KP / 12)
                .i(WristConstants.WRIST_KI / 12)
                .d(WristConstants.WRIST_KD / 12);

        config.softLimit
                .forwardSoftLimit(WristConstants.upperLimit)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(WristConstants.lowerLimit)
                .reverseSoftLimitEnabled(true);

        leftSparkMax.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        rightSparkMax.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        leftSparkMax.getEncoder().setPosition(-Math.PI / 2);

        rightSparkMax.getEncoder().setPosition(-Math.PI / 2);
    }

    private boolean shutdown = false;

    private double lastVoltageLeft = 0;

    private double lastVoltageRight = 0;

    private boolean usingVoltage = true;

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltageLeft = leftSparkMax.getBusVoltage() * leftSparkMax.getAppliedOutput();
        inputs.currentLeft = leftSparkMax.getOutputCurrent();
        inputs.temperatureLeft = leftSparkMax.getMotorTemperature();
        inputs.throughboreEncoderPositionLeft = throughboreEncoder.get() - 4.22;
        leftSparkMax.getEncoder().setPosition(inputs.throughboreEncoderPositionLeft);
        inputs.positionLeft = leftSparkMax.getEncoder().getPosition();
        inputs.throughboreConnectedLeft = throughboreEncoder.isConnected();

        if (inputs.temperatureLeft > 60) {
            shutdown = true;
        } else if (inputs.temperatureLeft < 58) {
            shutdown = false;
        }

        inputs.shutdownLeft = shutdown;

        if (inputs.throughboreEncoderPositionLeft >= WristConstants.upperLimit
                && (inputs.voltageLeft > 0 || lastVoltageLeft > 0)) {
            lastVoltageLeft = 0;
            usingVoltage = true;
        }
        if (inputs.throughboreEncoderPositionLeft <= WristConstants.lowerLimit
                && (inputs.voltageLeft < 0 || lastVoltageLeft < 0)) {
            lastVoltageLeft = 0;
            usingVoltage = true;
        }
        if (!inputs.throughboreConnectedLeft) {
            lastVoltageLeft = 0;
            usingVoltage = true;
        }
        if (shutdown) {
            lastVoltageLeft = 0;
            usingVoltage = true;
        }

        if (usingVoltage) {
            leftSparkMax.setVoltage(lastVoltageLeft);
        }

        inputs.voltageRight = rightSparkMax.getBusVoltage() * rightSparkMax.getAppliedOutput();
        inputs.currentRight = rightSparkMax.getOutputCurrent();
        inputs.temperatureRight = rightSparkMax.getMotorTemperature();
        inputs.throughboreEncoderPositionRight = throughboreEncoder.get() - 4.22;
        rightSparkMax.getEncoder().setPosition(inputs.throughboreEncoderPositionRight);
        inputs.positionRight = rightSparkMax.getEncoder().getPosition();
        inputs.throughboreConnectedRight = throughboreEncoder.isConnected();

        if (inputs.temperatureRight > 60) {
            shutdown = true;
        } else if (inputs.temperatureRight < 58) {
            shutdown = false;
        }

        inputs.shutdownRight = shutdown;

        if (inputs.throughboreEncoderPositionRight >= WristConstants.upperLimit
                && (inputs.voltageRight > 0 || lastVoltageRight > 0)) {
            lastVoltageRight = 0;
            usingVoltage = true;
        }
        if (inputs.throughboreEncoderPositionRight <= WristConstants.lowerLimit
                && (inputs.voltageRight < 0 || lastVoltageRight < 0)) {
            lastVoltageRight = 0;
            usingVoltage = true;
        }
        if (!inputs.throughboreConnectedRight) {
            lastVoltageRight = 0;
            usingVoltage = true;
        }
        if (shutdown) {
            lastVoltageRight = 0;
            usingVoltage = true;
        }

        if (usingVoltage) {
            rightSparkMax.setVoltage(lastVoltageRight);
        }
    }

    public void setVoltage(double voltage) {
        lastVoltageRight = voltage;
        lastVoltageLeft = voltage;
        usingVoltage = true;
    }

    public void setVoltageLeft(double voltage) {
        lastVoltageLeft = voltage;
        usingVoltage = true;
    }

    public void setVoltageRight(double voltage) {
        lastVoltageRight = voltage;
        usingVoltage = true;
    }

    public void setPositionControl(double reference) {
        rightSparkMax.getClosedLoopController().setReference(reference, ControlType.kPosition);
        leftSparkMax.getClosedLoopController().setReference(reference, ControlType.kPosition);
        usingVoltage = false;
    }
}
