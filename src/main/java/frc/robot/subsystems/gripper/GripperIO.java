package frc.robot.subsystems.gripper;

public interface GripperIO {
    public void updateInputs(GripperIOInputs inputs);

    public class GripperIOInputs {
        public double speed = 0;
        public double voltage = 0;
    }

    public void setVoltage(double voltage);
}
