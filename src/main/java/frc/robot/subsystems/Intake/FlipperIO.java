package frc.robot.subsystems.Intake;

public interface FlipperIO {

    public void updateInputs(FlipperIOInputs inputs);

    public class FlipperIOInputs {
        public double position = 0;
        public double voltage = 0;
    }

    public void setOpen();

    public void setClose();

    public void setVoltage(double voltage);
}
