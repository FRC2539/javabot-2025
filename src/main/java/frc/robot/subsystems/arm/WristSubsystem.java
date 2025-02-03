package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    public WristSubsystem(WristIO wristIO) {
        this.wristIO = wristIO;
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {
        wristIO.updateInputs(wristInputs);
        Logger.processInputs("RealOutputs/Wrist", wristInputs);
    }

    public Command turnWristRight() {
        return setVoltage(12);
    }

    public Command turnWristLeft() {
        return setVoltage(-12);
    }

    public Command setVoltage(double voltage) {
        return run(() -> wristIO.setVoltage(voltage));
    }

    public Command setPosition(double position) {
        return startRun();
    }

    public double getPosition() {
        return wristInputs.position;
    }
}
