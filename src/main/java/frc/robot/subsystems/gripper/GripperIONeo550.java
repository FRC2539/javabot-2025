package frc.robot.subsystems.gripper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.GripperConstants;

public class GripperIONeo550 implements GripperIO {
    private SparkMax leftSparkMax =
            new SparkMax(GripperConstants.leftMotorId, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightSparkMax =
            new SparkMax(GripperConstants.rightMotorId, SparkLowLevel.MotorType.kBrushless);

    private double lastVoltageLeft = 0;

    private double lastVoltageRight = 0;

    private AnalogInput sensor = new AnalogInput(GripperConstants.pieceSensorChannel);

    public GripperIONeo550() {
        SparkBaseConfig leftConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true);
        SparkBaseConfig rightConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        leftSparkMax.configure(leftConfig, null, PersistMode.kPersistParameters);
        rightSparkMax.configure(rightConfig, null, PersistMode.kPersistParameters);
    }

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltageLeft = leftSparkMax.getBusVoltage() * leftSparkMax.getAppliedOutput();
        inputs.currentLeft = leftSparkMax.getOutputCurrent();
        inputs.temperatureLeft = leftSparkMax.getMotorTemperature();

        inputs.voltageRight = rightSparkMax.getBusVoltage() * rightSparkMax.getAppliedOutput();
        inputs.currentRight = rightSparkMax.getOutputCurrent();
        inputs.temperatureRight = rightSparkMax.getMotorTemperature();

        leftSparkMax.setVoltage(lastVoltageLeft);
        rightSparkMax.setVoltage(lastVoltageRight);

        //System.out.println(sensor.getValue());
        inputs.hasPiece = sensor.getValue() < 50; 
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
