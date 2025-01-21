package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmPivotIO.ArmPivotIOInputs;
import frc.robot.subsystems.arm.WristIO.WristIOInputs;

import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    private ArmPivotIO pivotIO;
    private ArmPivotIOInputs armPivotInputs = new ArmPivotIOInputs();
    
    private WristIO wristIO;
    private WristIOInputs wristInputs = new WristIOInputs();
    

    public ArmSubsystem(ArmPivotIO armPivotIO, WristIO wristIO) {
        pivotIO = armPivotIO;
        wristIO = wristIO;
    }

    public void periodic() {

        
    }

    
}
