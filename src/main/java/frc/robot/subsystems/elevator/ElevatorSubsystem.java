package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs; 

public class ElevatorSubsystem extends SubsystemBase {
   
    private ElevatorIO piviotIO;
    private ElevatorIOInputs elevatorVoltage = new ElevatorIOInputs();
    
    private double voltage = 0;

    private double position = 0;

    private final double lowerLimit = 0;
    private final double upperLimit = 100;

    private Mechanism elevator;
    
    public ElevatorSubsystem(){
        this.piviotIO = piviotIO;
        this.elevator = elevator;
        
    }
    
    public void periodic(){
        

        this.piviotIO.updateInputs(elevatorVoltage);

        this.piviotIO.setVoltage(voltage);

        Logger.recordOutput("Elevator/Voltage", this.elevatorVoltage.voltage);
    }

    public Command zeroElevatorCommand(){
            return runOnce(() -> {
                piviotIO.setPosition(0);
            });

    }

    public Command moveElevatorUp(){
        return setVoltage(12).until(() -> elevatorVoltage.position >= upperLimit);
    }


    public Command setVoltage(double voltage) {
        return run (() -> {
            piviotIO.setVoltage(voltage);
        });
    }

    public Command setPosition(double position){
        return run(() -> {
            this.position = position;
        });
    }

    


    
    
}