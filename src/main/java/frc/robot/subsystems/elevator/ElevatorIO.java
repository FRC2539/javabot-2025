package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public interface ElevatorIO {

    public void updateInputs(ElevatorIOInputs inputs);

    public class ElevatorIOInputs {

        // public double voltage = 0;
        public double position = 0;
        public double speed = 0;
        public double voltage;



        public void setPosition(double position) {
            this.position = position;}

        public void setSpeed(double speed) {
            this.speed = speed;
        }

        public void setVoltage(double voltage){
            this.voltage = voltage;}

    }

    public void encoderUpdate();
    public PIDController getPIDController();

}



