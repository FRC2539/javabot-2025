package frc.robot.subsystems.gripper;

public class GripperIOSim implements GripperIO {
    public double voltage;
    public double speed;

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.speed = speed;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
