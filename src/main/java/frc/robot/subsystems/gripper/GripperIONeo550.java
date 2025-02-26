package frc.robot.subsystems.gripper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.WristConstants;

public class GripperIONeo550 implements GripperIO {
    private SparkMax leftSparkMax =
            new SparkMax(GripperConstants.leftMotorId, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightSparkMax =
            new SparkMax(GripperConstants.rightMotorId, SparkLowLevel.MotorType.kBrushless);

    private double lastVoltageLeft = -4;

    private double lastVoltageRight = 4;

    private DigitalInput sensor = new DigitalInput(GripperConstants.pieceSensorChannel);

    public GripperIONeo550() {
        SparkBaseConfig config =
                new SparkMaxConfig()
                        .idleMode(IdleMode.kBrake);

        leftSparkMax.configure(config, null, PersistMode.kPersistParameters);
        rightSparkMax.configure(config, null, PersistMode.kPersistParameters);
    }

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltageLeft = leftSparkMax.getBusVoltage() * leftSparkMax.getAppliedOutput();
        inputs.currentLeft = leftSparkMax.getOutputCurrent();
        inputs.temperatureLeft = leftSparkMax.getMotorTemperature();
        System.out.println(lastVoltageLeft);
        leftSparkMax.setVoltage(lastVoltageLeft);

        inputs.voltageRight = rightSparkMax.getBusVoltage() * rightSparkMax.getAppliedOutput();
        inputs.currentRight = rightSparkMax.getOutputCurrent();
        inputs.temperatureRight = rightSparkMax.getMotorTemperature();

        rightSparkMax.setVoltage(lastVoltageRight);

        inputs.hasPiece = sensor.get();
    }

    public void setVoltage(double voltage) {
        lastVoltageRight = voltage;
        lastVoltageLeft = voltage;
    }

    public void setVoltageLeft(double voltage) {
        lastVoltageLeft = voltage;
    }

    public void setVoltageRight(double voltage) {
        lastVoltageRight = voltage;
    }
}
