package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    private ElevatorIO piviotIO;
    private ElevatorIOInputs elevatorInputs = new ElevatorIOInputs();

    private double position = 0;

    private final double lowerLimit = 0;
    private final double upperLimit = 100;

    private Mechanism elevator;

    public ElevatorSubsystem(ElevatorIO elevatorIO, Mechanism elevator) {
        this.piviotIO = elevatorIO;
        this.elevator = elevator;
    }

    public void periodic() {

        piviotIO.updateInputs(elevatorInputs);

        Logger.recordOutput("Elevator/Voltage", elevatorInputs.voltage);

        if (elevatorInputs.voltage < 0 && elevatorInputs.position <= lowerLimit) {
            this.piviotIO.setVoltage(0);
        }

        if (elevatorInputs.voltage > 0 && elevatorInputs.position >= upperLimit) {
            this.piviotIO.setVoltage(0);
        }
    }

    public Command zeroElevatorCommand() {
        return runOnce(
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
                    this.position = position;
                });
    }
}
