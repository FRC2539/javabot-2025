package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class GripperIOSim implements GripperIO {
    public double voltage;
    public double speed;
    public LoggedNetworkBoolean gripperGPSensor =
            new LoggedNetworkBoolean("Gripper Sensor Sim", true);

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltageLeft = voltage;
        inputs.speedLeft = speed;
        inputs.sensorLeft = gripperGPSensor.get();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setVoltageLeft(double voltage) {}

    public void setVoltageRight(double voltage) {}
}
