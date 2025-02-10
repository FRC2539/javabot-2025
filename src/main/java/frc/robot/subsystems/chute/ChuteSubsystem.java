package frc.robot.subsystems.chute;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ChuteConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ChuteSubsystem extends SubsystemBase {
    private ChuteIO chuteIO;
    private ChuteIOInputsAutoLogged chuteInputs = new ChuteIOInputsAutoLogged();
    private LoggedNetworkNumber chuteTuneable = new LoggedNetworkNumber("chute tuneable", 0);

    private PIDController controller =
            new PIDController(
                    ChuteConstants.CHUTE_KP, ChuteConstants.CHUTE_KI, ChuteConstants.CHUTE_KD);

    private double reference = 0;

    private double upSetpoint = 10;
    private double downSetpoint = -10;

    public ChuteSubsystem(ChuteIO chuteIO) {
        controller.setTolerance(ChuteConstants.CHUTE_TOLERANCE);
        this.chuteIO = chuteIO;
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {
        chuteIO.updateInputs(chuteInputs);
        Logger.processInputs("RealOutputs/Chute", chuteInputs);
    }

    public Command tuneableVoltage() {
        return run(() -> setVoltage(chuteTuneable.get()));
    }

    public Command tunablePose() {
        return runOnce(() -> reference = chuteTuneable.get());
    }

    public Command setVoltage(double voltage) {
        return run(() -> chuteIO.setVoltage(voltage));
    }

    public Command movetoPosition(double position) {
        if (position > ChuteConstants.upperLimit) {
            position = ChuteConstants.upperLimit;
        }
        if (position < ChuteConstants.lowerLimit) {
            position = ChuteConstants.lowerLimit;
        }
        double nextPosition = position;
        return runOnce(
                () -> {
                    reference = nextPosition;
                });
    }

    public Command moveChuteUp() {
        return runOnce(() -> chuteIO.setPosition(upSetpoint));
    }

    public Command moveChuteDown() {
        return runOnce(() -> chuteIO.setPosition(downSetpoint));
    }

    //     private Command followReferenceThrubore() {
    //         return run(
    //                 () -> {
    //                     double voltage =
    //                             controller.calculate(
    //                                     wristInputs.throughboreEncoderPosition,
    //                                     isWristFlipped ? -reference : reference);
    //                     if (controller.atSetpoint()) {
    //                         voltage = 0;
    //                     } else {
    //                         voltage = Math.min(12.0, Math.max(-12.0, voltage)); // Clamp voltage
    //                     }
    //                     wristIO.setVoltage(voltage);
    //                 });
    //     }

    //     public double getPosition() {
    //         return wristInputs.throughboreEncoderPosition;
    //     }

    //     public double getFlippedPosition() {
    //         return isWristFlipped
    //                 ? -wristInputs.throughboreEncoderPosition
    //                 : wristInputs.throughboreEncoderPosition;
    //     }

    //     public double getInternalEncoderPosition() {
    //         return wristInputs.position;
    //     }

    //     public boolean isEncoderConnected() {
    //         return wristInputs.throughboreConnected;
    //     }
}
