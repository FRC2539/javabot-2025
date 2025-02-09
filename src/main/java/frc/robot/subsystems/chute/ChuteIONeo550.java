package frc.robot.subsystems.chute;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.ChuteConstants;

public class ChuteIONeo550 implements ChuteIO {
    private SparkMax chuteMotor =
            new SparkMax(ChuteConstants.CHUTE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    public ChuteIONeo550() {
        chuteMotor.getEncoder().setPosition(0);

        SparkBaseConfig config =
                new SparkMaxConfig()
                        .smartCurrentLimit((int) ChuteConstants.ChuteCurrent)
                        .secondaryCurrentLimit(ChuteConstants.ChuteCurrent)
                        .idleMode(IdleMode.kBrake);
        chuteMotor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean shutdown = false;

    private double lastVoltage = 0;

    public void updateInputs(ChuteIOInputs inputs) {
        inputs.position = chuteMotor.getEncoder().getPosition();
        inputs.voltage = chuteMotor.getBusVoltage() * chuteMotor.getAppliedOutput();
        inputs.current = chuteMotor.getOutputCurrent();
        inputs.temperature = chuteMotor.getMotorTemperature();
        // inputs.throughboreEncoderPosition = throughboreEncoder.get();
        // inputs.throughboreConnected = throughboreEncoder.isConnected();

        // if (inputs.temperature > 60) {
        //     shutdown = true;
        // } else if (inputs.temperature < 58) {
        //     shutdown = false;
        // }

        // inputs.shutdown = shutdown;

        // if (inputs.throughboreEncoderPosition >= ChuteConstants.upperLimit && lastVoltage > 0) {
        //     lastVoltage = 0;
        // }
        // if (inputs.throughboreEncoderPosition <= ChuteConstants.lowerLimit && lastVoltage < 0) {
        //     lastVoltage = 0;
        // }
        // if (!inputs.throughboreConnected) {
        //     lastVoltage = 0;
        // }
        if (shutdown) {
            lastVoltage = 0;
        }

        chuteMotor.setVoltage(lastVoltage);
    }

    public void setVoltage(double voltage) {
        lastVoltage = voltage;
    }
}
