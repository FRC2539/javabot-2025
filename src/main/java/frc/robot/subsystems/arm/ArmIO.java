package frc.robot.subsystems.arm;

public interface ArmIO {

    public void updateInputs(ArmIOInputs inputs);

    public class ArmIOInputs {

        public double position = 0;
        public double speed = 0;
        public double voltage;
    }

    public void setArmPosition(double position);

    public void setArmVoltage(double voltage);

    public void setWristPosition(double position);

    public void setWristVoltage(double voltage);
}
