package frc.robot.subsystems.TestBase;

public interface MotorIO {
    public void updateInputs(MotorIOInputs inputs);

    public class MotorIOInputs {
        public double speed = 0;
        public double voltage = speed*12;
        
    }

    public void setMotorSpeed(double speeds);
}


