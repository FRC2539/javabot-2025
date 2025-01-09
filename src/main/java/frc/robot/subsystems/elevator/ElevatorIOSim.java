package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
    private double position;
    private double voltage;

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = position;
        
        
    }

    public void setVoltage(double voltage) {
        
    }
    public void setPosition(double position) {

    }
    

}
