package frc.robot.subsystems.gripper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.constants.GripperConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class GripperIONeo550 implements GripperIO {
    private SparkMax leftSparkMax =
            new SparkMax(GripperConstants.leftMotorId, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightSparkMax =
            new SparkMax(GripperConstants.rightMotorId, SparkLowLevel.MotorType.kBrushless);

    private double lastVoltageLeft = 0;

    private double lastVoltageRight = 0;

    private AnalogInput initialPieceSensor = new AnalogInput(GripperConstants.initialSensorChannel);

    private DigitalInput hasPieceSensor = new DigitalInput(GripperConstants.secondSensorChannel);

    public GripperIONeo550() {
        SparkBaseConfig leftConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true);
        SparkBaseConfig rightConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        Shuffleboard.getTab("debug")
                .addInteger("first sensor value", () -> initialPieceSensor.getValue());
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

        // System.out.println(sensor.getValue());
        inputs.firstSensor = initialPieceSensor.getValue() < 2800;

        // inputs.secondSensor = hasPieceSensor.getValue() < 50;
        inputs.secondSensor = !hasPieceSensor.get();
    }

    public void setVoltage(double voltage) {
        lastVoltageRight = voltage;
        lastVoltageLeft = voltage;
    }

    @AutoLogOutput
    public int getFirstSensorValue() {
        return initialPieceSensor.getValue();
    }

    // @AutoLogOutput
    // public boolean getSecondSensorValue() {
    //     return hasPieceSensor.get();
    // }

    public void setVoltageLeft(double voltage) {
        lastVoltageLeft = voltage;
    }

    public void setVoltageRight(double voltage) {
        lastVoltageRight = voltage;
    }
}
