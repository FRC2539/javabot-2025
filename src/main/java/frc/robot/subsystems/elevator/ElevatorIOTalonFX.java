package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX; 

public class ElevatorIOTalonFX implements ElevatorIO {
    // idk ids yet
    private final TalonFX elevatorLeader = new TalonFX(41, "CANivore");
    private final TalonFX elevatorFollower = new TalonFX(42, "CANivore");
    public final void follower(){
        elevatorFollower.setControl(new Follower(42, false));
    }

    public ElevatorIOTalonFX(){
        elevatorLeader.setPosition(0);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        
        inputs.position = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeader.getPosition().refresh().getValueAsDouble();
        
    }

    public void setVoltage(double voltage) {
        elevatorLeader.setVoltage(voltage);
    }

    public void setPosition(double position) {
        elevatorLeader.setPosition(position);
    }

    

}
