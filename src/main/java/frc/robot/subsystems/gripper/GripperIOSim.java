package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class GripperIOSim implements GripperIO {
    private double leftVoltage;
    private double rightVoltage;
    private double speed;
    private LoggedNetworkBoolean gripperGPSensorInitial =
            new LoggedNetworkBoolean("Gripper Initial Sensor Sim", true);

    private LoggedNetworkBoolean gripperGPSensorSecond =
            new LoggedNetworkBoolean("Gripper Second Sensor Sim", true);

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltageLeft = leftVoltage;
        inputs.voltageRight = rightVoltage;
        inputs.speedLeft = speed;
        inputs.firstSensor = gripperGPSensorInitial.get();
        inputs.secondSensor = gripperGPSensorSecond.get();
    }

    public void setVoltage(double voltage) {
        this.leftVoltage = voltage;
        this.rightVoltage = voltage;
    }

    public void setVoltageLeft(double voltage) {
        this.leftVoltage = voltage;
    }

    public void setVoltageRight(double voltage) {
        this.rightVoltage = voltage;
    }
}
