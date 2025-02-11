package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface FlipperIO {

    public void resetPosition(double position);

    public void updateInputs(FlipperIOInputs inputs);

    @AutoLog
    public class FlipperIOInputs {
        public double position = 0;
        public double voltage = 0;
        public double temperature = 0;
        public double current = 0;
    }

    public void setOpen();

    public void setClose();

    public void setVoltage(double voltage);
}
