package frc.robot.subsystems.Arm;

public interface ArmrollerIO {
    public void updateInputs(ArmrollerIO inputs);

    public class ArmrollerIOInputs {
        public double speed = 0;
        public double voltage;
    }

    public void setVoltage(double voltage);
}
