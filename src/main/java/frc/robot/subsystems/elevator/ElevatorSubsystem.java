package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    private ElevatorIO piviotIO;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    // LoggedNetworkNumber elevatorPosition = new LoggedNetworkNumber("Elevator Position", 0);

    private final double lowerLimit = 0;
    private final double upperLimit = 100;

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.piviotIO = elevatorIO;
        setDefaultCommand(setPosition(0));
    }

    public void periodic() {

        piviotIO.updateInputs(elevatorInputs);

        Logger.processInputs("RealOutputs/Elevator", elevatorInputs);

        if (elevatorInputs.voltage < 0 && elevatorInputs.position <= lowerLimit) {
            this.piviotIO.setVoltage(0);
        }

        if (elevatorInputs.voltage > 0 && elevatorInputs.position >= upperLimit) {
            this.piviotIO.setVoltage(0);
        }
    }

    // public Command elevatorTuneable() {
    //     return run(
    //             () -> {
    //                 double position = elevatorPosition.get();
    //                 piviotIO.setPosition(position);
    //             });
    // }

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

    public double getPosition() {
        return piviotIO.getPosition();
    }
}
