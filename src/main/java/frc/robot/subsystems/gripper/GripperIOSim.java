package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class GripperIOSim implements GripperIO {
    public double voltage;
    public double speed;
    public LoggedNetworkBoolean gripperGPSensor =
            new LoggedNetworkBoolean("Gripper Sensor Sim", true);

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.speed = speed;
        inputs.sensor = gripperGPSensor.get();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
