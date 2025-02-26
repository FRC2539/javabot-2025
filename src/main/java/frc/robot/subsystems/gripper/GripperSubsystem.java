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
                .andThen(Commands.waitUntil(HAS_PIECE))
                .andThen(setVoltage(0)); // TODO: I dont think i need to set the voltage to zero again but i am anyways
    }

    public Command placePiece() {
        return setVoltage(GripperConstants.placeVoltage)
                .andThen(Commands.waitUntil(HAS_PIECE))
                .andThen(Commands.waitSeconds(0.01))
                .andThen(setVoltage(0));
    }

    public Command ejectReverse(double voltage) {
        return setVoltage(-voltage);
    }

    public Command setVoltage(double voltage) {
        return Commands.runOnce(
                () -> {
                    gripperIO.setVoltage(voltage);
                },
                this);
    }

    public boolean hasPiece() {
        return false;
    }
}
