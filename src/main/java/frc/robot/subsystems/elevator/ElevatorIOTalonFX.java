package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX; 

public class ElevatorIOTalonFX implements ElevatorIO {
    // idk ids yet
    private final TalonFX elevatorLeader = new TalonFX(98, "CANivore");
    private final TalonFX elevatorFollower = new TalonFX(99, "CANivore");

    


    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = elevatorFollower.getPosition().refresh().getValueAsDouble();
        inputs.position = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorFollower.getPosition().refresh().getValueAsDouble();
        
    }

    public void setVoltage(double voltage) {
        elevatorLeader.setVoltage(voltage);
    }


    
    public void setPosition(double position) {
        elevatorLeader.setPosition(position);
    }

}
