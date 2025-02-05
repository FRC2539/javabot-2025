package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    public void updateInputs(WristIOInputs inputs);

    @AutoLog
    public class WristIOInputs {
        public double position = 0;
        public double temperature = 0;
        public double current = 0;
        public double voltage = 0;
        public double throughboreEncoderPosition = 0;
    }

    public void setVoltage(double voltage);
}
