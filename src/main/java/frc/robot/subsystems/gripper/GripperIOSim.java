package frc.robot.subsystems.gripper;

public class GripperIOSim implements GripperIO {
    private double voltage;
    private double speed;

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.speed = speed;
        inputs.temperature = 0;
        inputs.current = 0;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
