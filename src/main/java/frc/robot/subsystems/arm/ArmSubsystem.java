package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private ArmPivotIO pivotIO;
    private ArmPivotIOInputsAutoLogged armPivotInputs = new ArmPivotIOInputsAutoLogged();

    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    public ArmSubsystem(ArmPivotIO armPivotIO, WristIO swristIO) {
        pivotIO = armPivotIO;
        wristIO = swristIO;
    }

    public void periodic() {
        pivotIO.updateInputs(armPivotInputs);
        wristIO.updateInputs(wristInputs);

        Logger.processInputs("RealOutputs/Arm", armPivotInputs);
        Logger.processInputs("RealOutputs/Wrist", wristInputs);
        
    }

    public Command setVoltageArm(double voltage){
        return run(() -> {
            pivotIO.setVoltage(voltage);
        });
    }

    public Command setVoltageWrist(double voltage){
        return run(() -> {
            wristIO.setVoltage(voltage);
        });
    }

}
