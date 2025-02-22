package frc.robot.subsystems.chute;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ChuteConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ChuteSubsystem extends SubsystemBase {
    private ChuteIO chuteIO;
    private ChuteIOInputsAutoLogged chuteInputs = new ChuteIOInputsAutoLogged();
    private LoggedNetworkNumber chuteTuneable = new LoggedNetworkNumber("chute tuneable", 0);

    @AutoLogOutput private boolean isUp = false;
    public final Trigger UP = new Trigger(() -> isUp);

    @AutoLogOutput private boolean isDown = false;
    public final Trigger DOWN = new Trigger(() -> isDown);

    private PIDController controller =
            new PIDController(
                    ChuteConstants.CHUTE_KP, ChuteConstants.CHUTE_KI, ChuteConstants.CHUTE_KD);

    // private double reference = 0;

    // private double upSetpoint = 10;
    // private double downSetpoint = -10;

    public ChuteSubsystem(ChuteIO chuteIO) {
        controller.setTolerance(ChuteConstants.CHUTE_TOLERANCE);
        this.chuteIO = chuteIO;
        setDefaultCommand(Commands.either(moveChuteDown(), moveChuteUp(), () -> lastSetDown));
    }

    public void periodic() {
        chuteIO.updateInputs(chuteInputs);
        Logger.processInputs("RealOutputs/Chute", chuteInputs);
    }

    public Command tuneableVoltage() {
        return run(() -> setVoltage(chuteTuneable.get()));
    }

    // public Command tunablePose() {
    //     return runOnce(() -> reference = chuteTuneable.get());
    // }

    public Command setVoltage(double voltage) {
        return run(() -> chuteIO.setVoltage(voltage));
    }

    // public Command movetoPosition(double position) {
    //     if (position > ChuteConstants.upperLimit) {
    //         position = ChuteConstants.upperLimit;
    //     }
    //     if (position < ChuteConstants.lowerLimit) {
    //         position = ChuteConstants.lowerLimit;
    //     }
    //     double nextPosition = position;
    //     return runOnce(
    //             () -> {
    //                 reference = nextPosition;
    //             });
    // }

    public final Trigger STALLING =
            new Trigger(() -> chuteInputs.current >= ChuteConstants.ChuteCurrentTrigger);

    private boolean lastSetDown = false;

    public Command moveChuteUp() {
        return setVoltage(-4)
                .withTimeout(0.05)
                .andThen(setVoltage(-4).until(STALLING))
                .andThen(setUp())
                .andThen(setVoltage(-1))
                .beforeStarting(
                        () -> {
                            isDown = false;
                            lastSetDown = false;
                        });
    }

    public Command moveChuteDown() {
        return (setVoltage(1.5)
                        .withTimeout(0.05)
                        .andThen(
                                setVoltage(1.5)
                                        .until(STALLING)
                                        .andThen(setDown())
                                        .andThen(setVoltage(0.0))))
                .beforeStarting(
                        () -> {
                            isUp = false;
                            lastSetDown = true;
                        });
    }

    // public void setUp() {
    //     isUp = true;
    //     isDown = false;
    // }

    // public void setDown() {
    //     isUp = false;
    //     isDown = true;
    // }

    public Command setUp() {
        return runOnce(
                () -> {
                    isUp = true;
                    isDown = false;
                });
    }

    public Command setDown() {
        return runOnce(
                () -> {
                    isUp = false;
                    isDown = true;
                });
    }

    // private Command followReferenceThrubore() {
    //     return run(
    //             () -> {
    //                 double voltage =
    //                         controller.calculate(
    //                                 wristInputs.throughboreEncoderPosition,
    //                                 isWristFlipped ? -reference : reference);
    //                 if (controller.atSetpoint()) {
    //                     voltage = 0;
    //                 } else {
    //                     voltage = Math.min(12.0, Math.max(-12.0, voltage)); // Clamp voltage
    //                 }
    //                 wristIO.setVoltage(voltage);
    //             });
    // }

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
