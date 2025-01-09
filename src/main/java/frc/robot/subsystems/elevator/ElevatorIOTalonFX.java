package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX; 

public class ElevatorIOTalonFX implements ElevatorIO {
    // idk ids yet
    private TalonFX elevatorLeader = new TalonFX(98, "CANivore");
    private TalonFX elevatorFollower = new TalonFX(99, "CANivore");


    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = elevatorFollower.getPosition().refresh().getValueAsDouble();
        inputs.position = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorFollower.getPosition().refresh().getValueAsDouble();
        
    }

    public void setVoltage(double voltage) {
        elevatorLeader.setVoltage(voltage);
        elevatorFollower.setVoltage(voltage);
    }


    @Override
    public void setPosition(double position) {
        //Figure out how to make it so we can see how far we need to go to get to a certain position.(?)
    }

}
