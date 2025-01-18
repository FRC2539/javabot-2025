package frc.robot.subsystems.intake;

public interface IntakeRollerIO {
    public void updateInputs(IntakeRollerIOInputs inputs);

    public class IntakeRollerIOInputs {
        public double speed = 0;
        public double voltage = 0;
    }

    public void setVoltage(double voltage);
}
