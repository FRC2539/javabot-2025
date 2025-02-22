package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.ClimberConstants;

public class ClimberHeadIONeo550 implements ClimberHeadIO {
    private SparkMax climberHeadMotor =
            new SparkMax(ClimberConstants.CLIMBER_HEAD_ID, SparkLowLevel.MotorType.kBrushless);

    public ClimberHeadIONeo550() {
        climberHeadMotor.getEncoder().setPosition(0);

        SparkBaseConfig config =
                new SparkMaxConfig()
                        .smartCurrentLimit((int) ClimberConstants.ClimberHeadCurrent)
                        .secondaryCurrentLimit(ClimberConstants.ClimberHeadCurrent)
                        .idleMode(IdleMode.kBrake);
        climberHeadMotor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean shutdown = false;

    private double lastVoltage = 0;

    public void updateInputs(ClimberHeadIOInputs inputs) {
        inputs.position = climberHeadMotor.getEncoder().getPosition();
        inputs.voltage = climberHeadMotor.getBusVoltage() * climberHeadMotor.getAppliedOutput();
        inputs.current = climberHeadMotor.getOutputCurrent();
        inputs.temperature = climberHeadMotor.getMotorTemperature();

        if (inputs.temperature > 60) {
            shutdown = true;
        } else if (inputs.temperature < 58) {
            shutdown = false;
        }

        inputs.shutdown = shutdown;

        climberHeadMotor.setVoltage(lastVoltage);
    }

    public void setVoltage(double voltage) {
        lastVoltage = voltage;
    }
}
