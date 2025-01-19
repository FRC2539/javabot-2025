package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GripperSubsystem extends SubsystemBase {

    private final GripperIO gripperIO;

    private final GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();

    public GripperSubsystem(GripperIO io) {
        this.gripperIO = io;
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {
        gripperIO.updateInputs(gripperInputs);
        Logger.processInputs("RealOutputs/Gripper", gripperInputs);
    }

    public Command intakeSpin() {
        return setVoltage(12);
    }

    public Command ejectSpin() {
        return setVoltage(-12);
    }

    public Command setVoltage(double voltage) {
        return run(
                () -> {
                    gripperIO.setVoltage(voltage);
                });
    }
}
