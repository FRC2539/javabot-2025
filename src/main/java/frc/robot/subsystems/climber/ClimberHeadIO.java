package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberHeadIO {

    public void updateInputs(ClimberHeadIOInputs inputs);

    @AutoLog
    public class ClimberHeadIOInputs {
        public double position = 0;
        public double temperature = 0;
        public double current = 0;
        public double voltage = 0;
        public boolean shutdown = false;
    }

    public void setVoltage(double voltage);
}
