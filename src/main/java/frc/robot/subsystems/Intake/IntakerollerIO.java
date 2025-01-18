package frc.robot.subsystems.Intake;

public interface IntakerollerIO {
    public void updateInputs(IntakerollerIOInputs inputs);

    public class IntakerollerIOInputs {
        public double speed = 0;
        public double voltage;
    }

    public void setVoltage(double voltage);
}
