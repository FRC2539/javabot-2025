package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    public void updateInputs(ElevatorIOInputs inputs);

    public class ElevatorIOInputs {
        
        //public double voltage = 0;
        public double position = 0;
        public double speed = 0;
        public double voltage;

    }

    public void setVoltage(double voltage);
    public void setPosition(double position);
    

}