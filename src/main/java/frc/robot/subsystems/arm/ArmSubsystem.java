package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    private ArmPivotIO armPivotIO;
    private ArmPivotIOInputsAutoLogged armPivotInputs = new ArmPivotIOInputsAutoLogged();

    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    public ArmSubsystem(ArmPivotIO armPivotIO, WristIO wristIO) {
        this.armPivotIO = armPivotIO;
        this.wristIO = wristIO;

        setDefaultCommand(
                run(
                        () -> {
                            armPivotIO.setPosition(0);
                            wristIO.setPosition(0);
                        }));
    }

    public void periodic() {
        armPivotIO.updateInputs(armPivotInputs);
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
                    armPivotIO.setVoltage(voltage);
                });
    }

    public Command setVoltageWrist(double voltage) {
        return run(
                () -> {
                    wristIO.setVoltage(voltage);
                });
    }

    public Command setPosition(double wrist, double arm) {
        return run(
                () -> {
                    armPivotIO.setPosition(arm);
                    wristIO.setPosition(wrist);
                });
    }

    public double getArmPosition() {
        return armPivotIO.getPosition();
    }

    public double getWristPosition() {
        return wristIO.getPosition();
    }
}
