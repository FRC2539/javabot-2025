package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class GripperSubsystem extends SubsystemBase {

    private GripperIO piviotIO;

    private GripperIOInputsAutoLogged armrollerInputs = new GripperIOInputsAutoLogged();

    public final Trigger HAS_PIECE = new Trigger(this::hasPiece);

    // NetworkTableInstance nInstance = NetworkTableInstance.getDefault();
    // NetworkTable table = nInstance.getTable("SmartDashboard");
    // NetworkTableValue grippervoltage = table.getValue("grippervoltage");

    LoggedNetworkNumber grippervoltage = new LoggedNetworkNumber("Gripper Voltage", 0);

    public GripperSubsystem(GripperIO armrollerIO) {
        this.piviotIO = armrollerIO;
        setDefaultCommand(Commands.either(holdCoral(), setVoltage(0), HAS_PIECE));
    }

    public void periodic() {
        piviotIO.updateInputs(armrollerInputs);
        Logger.processInputs("RealOutputs/Gripper", armrollerInputs);
    }

    public Command gripperTuneable() {
        return run(
                () -> {
                    double voltage = grippervoltage.get();
                    piviotIO.setVoltage(voltage);
                });
    }

    public Command intakeSpinCoral() {
        return setVoltage(12);
    }

    public Command holdCoral() { 
        return setVoltage(0.25);
    }

    public Command ejectSpinCoral() {
        return setVoltage(-1);
    }

    public Command intakeSpinAlgae() {
        return setVoltage(12);
    }

    public Command ejectSpinAlgae() {
        return setVoltage(-12);
    }

    public Command slowEjectSpinAlgae() {
        return setVoltage(-3);
    }

    public Command setVoltage(double voltage) {
        return run(
                () -> {
                    piviotIO.setVoltage(voltage);
                });
    }

    public boolean hasPiece() {
        return armrollerInputs.sensor;
    }

    private boolean hasAlgae = false;

    public void setHasAlgae(boolean hasAlgae) {
        hasAlgae = true;
    }
}
