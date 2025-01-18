package frc.robot.subsystems.Intake;

public interface IntakerollerIO {
    public void updateInputs(IntakerollerIO inputs);

    public class IntakerollerInputs {
        public double speed = 0;
        public double voltage;
    }

    public void setVoltage(double voltage);
}
