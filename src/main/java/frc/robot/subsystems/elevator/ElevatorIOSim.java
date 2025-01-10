package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = position;
        inputs.voltage = voltage;
        
        
    }

    public void setVoltage(double voltage) {
        
    }
    public void setPosition(double position) {

    }
    

}
