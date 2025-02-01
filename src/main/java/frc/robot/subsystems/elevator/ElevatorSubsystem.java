package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    private ElevatorIO piviotIO;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.piviotIO = elevatorIO;
        setDefaultCommand(setPosition(0));
    }

    public void periodic() {

        piviotIO.updateInputs(elevatorInputs);

        Logger.processInputs("RealOutputs/Elevator", elevatorInputs);
    }

    public Command zeroElevatorCommand() {
        return runOnce(
                () -> {
                    piviotIO.resetPosition(0);
                });
    }

    public Command moveElevatorUp() {
        return setVoltage(12);
    }

    public Command moveElevatorDown() {
        return setVoltage(-12);
    }

    public Command setVoltage(double voltage) {
        return run(
                () -> {
                    piviotIO.setVoltage(voltage);
                });
    }

    public Command setPosition(double position) {
        return run(
                () -> {
                    piviotIO.setPosition(position);
                });
    }

    public double getPosition() {
        return elevatorInputs.position;
    }
}
