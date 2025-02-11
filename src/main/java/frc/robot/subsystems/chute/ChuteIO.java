package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.AutoLog;

public interface ChuteIO {
    public void updateInputs(ChuteIOInputs inputs);

    @AutoLog
    public class ChuteIOInputs {
        public double position = 0;
        public double temperature = 0;
        public double current = 0;
        public double voltage = 0;
        public boolean shutdown = false;
        // public double throughboreEncoderPosition = 0;
        // public boolean throughboreConnected = false;
    }

    public void setVoltage(double voltage);
}
