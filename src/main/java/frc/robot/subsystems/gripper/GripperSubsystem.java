package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GripperConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class GripperSubsystem extends SubsystemBase {

    private GripperIO gripperIO;

    private GripperIOInputsAutoLogged gripperInputs = new GripperIOInputsAutoLogged();

    public final Trigger HAS_PIECE = new Trigger(this::hasPiece);
    public final Trigger PIECE_SEATED = new Trigger(this::isPieceSeated);
    

    LoggedNetworkNumber leftGripperVoltage =
            new LoggedNetworkNumber("Left Gripper Motor Voltage", 0);
    LoggedNetworkNumber rightGripperVoltage =
            new LoggedNetworkNumber("Right Gripper Motor Voltage ", 0); // TODO: NAME?

    public GripperSubsystem(GripperIO armrollerIO) {
        this.gripperIO = armrollerIO;
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {
        gripperIO.updateInputs(gripperInputs);
        Logger.processInputs("RealOutputs/Gripper", gripperInputs);
    }

    public Command gripperLeftTuneable() {
        return run(
                () -> {
                    double voltage = leftGripperVoltage.get();
                    gripperIO.setVoltage(voltage);
                });
    }

    public Command gripperRightTuneable() {
        return run(
                () -> {
                    double voltage = rightGripperVoltage.get();
                    gripperIO.setVoltage(voltage);
                });
    }

    public Command intake(double voltage) {
        return setVoltage(voltage);
    }

    public Command intakeUntilPiece() {
        return setVoltage(GripperConstants.handoffVoltage)
        .until(() -> gripperInputs.firstSensor)
        .andThen(setVoltage(GripperConstants.slowHandoffVoltage)
        .until(() -> gripperInputs.secondSensor));
    }

    public Command placePiece() {
        return setVoltage(GripperConstants.placeVoltage)
                .until(HAS_PIECE.negate())
                .andThen(Commands.waitSeconds(0.3))
                .withTimeout(4);
    }

    public Command placePieceReverse() {
        return setVoltage(1, 4)
                .until(HAS_PIECE.negate())
                .andThen(Commands.waitSeconds(0.3))
                .withTimeout(4);
    }

    public Command ejectReverse(double voltage) {
        return setVoltage(-voltage);
    }

    public Command setVoltage(double voltage) {
        return Commands.run(
                () -> {
                    gripperIO.setVoltage(voltage);
                },
                this);
    }

    public Command setVoltage(double leftVoltage, double rightVoltage) {
        return Commands.run(
                () -> {
                    gripperIO.setVoltageLeft(leftVoltage);
                    gripperIO.setVoltageRight(rightVoltage);
                },
                this);
    }

    public boolean isPieceSeated() {
        return gripperInputs.firstSensor;
    }

    public boolean hasPiece() {
        return gripperInputs.secondSensor;
    }

    public boolean intaking() {
        return gripperInputs.voltageLeft > 3;
    }
}
