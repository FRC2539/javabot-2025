package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WristSubsystem extends SubsystemBase {
    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    private LoggedNetworkNumber wristTuneable = new LoggedNetworkNumber("wrist tuneable", 0);

    private PIDController controller = new PIDController(0.1, 0, 0);

    private double reference = 0;

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

    public Command tuneable() {
        return run(() -> setVoltage(wristTuneable.get()));
    }

    public Command setVoltage(double voltage) {
        return run(() -> wristIO.setVoltage(voltage));
    }

    public Command setPosition(double position) {
        return startRun(
                () -> {
                    reference = position;
                },
                () -> {
                    double voltage =
                            controller.calculate(wristInputs.throughboreEncoderPosition, reference);
                    voltage = Math.min(12.0, Math.max(-12.0, voltage)); // Clamp voltage
                    wristIO.setVoltage(voltage);
                });
    }

    public double getPosition() {
        return wristInputs.position;
    }
}
