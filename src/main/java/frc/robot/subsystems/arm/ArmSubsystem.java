package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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

        wristIO.encoderUpdate();
        Logger.processInputs("RealOutputs/Arm", armPivotInputs);
        Logger.processInputs("RealOutputs/Wrist", wristInputs);
    }

    public Command turnWristRight() {
        return setVoltageWrist(12);
    }

    public Command turnWristLeft() {
        return setVoltageWrist(-12);
    }

    public Command armPivotUp() {
        return setVoltageArm(12);
    }

    public Command armpivotDown() {
        return setVoltageArm(-12);
    }

    public Command setVoltageArm(double voltage) {
        return run(
                () -> {
                    pivotIO.setVoltage(voltage);
                });
    }

    public Command setVoltageWrist(double voltage) {
        return run(
                () -> {
                    wristIO.setVoltage(voltage);
                });
    }
}
