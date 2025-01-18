package frc.robot.subsystems.gripper;

public class GripperIOSim implements GripperIO {
    public double voltage;
    public double speed;

    public void updateInputs(GripperIOInputs inputs) {
        voltage = inputs.voltage;
        speed = inputs.speed;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
