package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
    public void updateInputs(GripperIOInputs inputs);

    @AutoLog
    public class GripperIOInputs {
        public double speed = 0;
        public double voltage = 0;
        public double temperature = 0;
        public double current = 0;
    }

    public void setVoltage(double voltage);
}
