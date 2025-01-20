package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    private ElevatorIO piviotIO;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private final double lowerLimit = 0;
    private final double upperLimit = 100;

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.piviotIO = elevatorIO;
    }

    public void periodic() {

        piviotIO.updateInputs(elevatorInputs);

        Logger.processInputs("Realoutputs/Elevator", elevatorInputs);

        if (elevatorInputs.voltage < 0 && elevatorInputs.position <= lowerLimit) {
            this.piviotIO.setVoltage(0);
        }

        if (elevatorInputs.voltage > 0 && elevatorInputs.position >= upperLimit) {
            this.piviotIO.setVoltage(0);
        }
    }

    public Command zeroElevatorCommand() {
        return run(
                () -> {
                    piviotIO.setPosition(0);
                });
    }

    public Command moveElevatorUp() {
        return setVoltage(12).until(() -> elevatorInputs.position >= upperLimit);
    }

    public Command moveElevatorDown() {
        return setVoltage(-12).until(() -> elevatorInputs.position <= lowerLimit);
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
}
