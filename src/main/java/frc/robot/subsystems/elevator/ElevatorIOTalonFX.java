package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX; 

public class ElevatorIOTalonFX implements ElevatorIO {
    // idk ids yet
    private TalonFX leadElevatorMotor = new TalonFX(99);
    private TalonFX followerElevatorMotor = new TalonFX(98);

    public ElevatorIOTalonFX() {
        
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        
    }

}
