package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    public void updateInputs(KickerIOInputs inputs);

    @AutoLog
    public class KickerIOInputs {
        public double voltage = 0;
        public double position;
    }

    public void setVoltage(double voltage);
}
