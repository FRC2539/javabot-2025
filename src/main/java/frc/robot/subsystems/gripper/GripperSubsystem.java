package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class GripperSubsystem extends SubsystemBase {

    private GripperIO piviotIO;

    private GripperIOInputsAutoLogged armrollerInputs = new GripperIOInputsAutoLogged();

    // NetworkTableInstance nInstance = NetworkTableInstance.getDefault();
    // NetworkTable table = nInstance.getTable("SmartDashboard");
    // NetworkTableValue grippervoltage = table.getValue("grippervoltage");

    LoggedNetworkNumber grippervoltage = new LoggedNetworkNumber("Gripper Voltage");

    public GripperSubsystem(GripperIO armrollerIO) {
        this.piviotIO = armrollerIO;
        setDefaultCommand(setVoltage(0));
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

    public Command ejectSpinCoral() {
        return setVoltage(-12);
    }

    public Command intakeSpinAlgae() {
        return setVoltage(12);
    }

    public Command ejectSpinAlgae() {
        return setVoltage(-12);
    }

    public Command setVoltage(double voltage) {
        return run(
                () -> {
                    piviotIO.setVoltage(voltage);
                });
    }
}
