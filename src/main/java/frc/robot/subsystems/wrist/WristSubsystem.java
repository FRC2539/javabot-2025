package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WristSubsystem extends SubsystemBase {
    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    private LoggedNetworkNumber wristTuneable = new LoggedNetworkNumber("wrist tuneable", 0);

    private PIDController controller =
            new PIDController(
                    WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);

    private double reference = 0;

    private boolean isWristFlipped = false;

    public WristSubsystem(WristIO wristIO) {
        controller.setTolerance(WristConstants.WRIST_TOLERANCE);
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

    public Command tuneableVoltage() {
        return run(() -> setVoltage(wristTuneable.get()));
    }

    public Command tunablePose() {
        return runOnce(() -> reference = wristTuneable.get()).andThen(followReferenceThrubore());
    }

    public Command setVoltage(double voltage) {
        return run(() -> wristIO.setVoltage(voltage));
    }

    public Command setPosition(double position) {
        if (position > WristConstants.upperLimit) {
            position = WristConstants.upperLimit;
        }
        if (position < WristConstants.lowerLimit) {
            position = WristConstants.lowerLimit;
        }
        double nextPosition = position;
        return runOnce(
                        () -> {
                            reference = nextPosition;
                        })
                .andThen(followReferenceThrubore());
    }

    public Command flipWristPosition() {
        return Commands.runOnce(
                () -> {
                    isWristFlipped = !isWristFlipped;
                });
    }

    private Command followReferenceThrubore() {
        return run(
                () -> {
                    double voltage =
                            controller.calculate(
                                    wristInputs.throughboreEncoderPosition,
                                    isWristFlipped ? -reference : reference);
                    if (controller.atSetpoint()) {
                        voltage = 0;
                    } else {
                        voltage = Math.min(12.0, Math.max(-12.0, voltage)); // Clamp voltage
                    }
                    wristIO.setVoltage(voltage);
                });
    }

    public double getPosition() {
        return wristInputs.throughboreEncoderPosition;
    }

    public double getFlippedPosition() {
        return isWristFlipped
                ? -wristInputs.throughboreEncoderPosition
                : wristInputs.throughboreEncoderPosition;
    }

    public double getInternalEncoderPosition() {
        return wristInputs.position;
    }

    public boolean isEncoderConnected() {
        return wristInputs.throughboreConnected;
    }
}
