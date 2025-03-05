package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    private ElevatorIO piviotIO;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    // LoggedNetworkNumber elevatorPosition = new LoggedNetworkNumber("Elevator Position", 0);

    private SysIdRoutine elevatorSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            state ->
                                    Logger.recordOutput(
                                            "Elevator/SysIdElevator_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (voltage) -> piviotIO.setVoltage(voltage.in(Volts)), null, this));

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.piviotIO = elevatorIO;
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {

        piviotIO.updateInputs(elevatorInputs);

        Logger.processInputs("RealOutputs/Elevator", elevatorInputs);
    }

    public Command runQStaticElevatorSysId(SysIdRoutine.Direction direction) {
        return elevatorSysIdRoutine.quasistatic(direction);
    }

    public Command runDynamicElevatorSysId(SysIdRoutine.Direction direction) {
        return elevatorSysIdRoutine.dynamic(direction);
    }

    // public Command elevatorTuneable() {
    //     return run(
    //             () -> {
    //                 double position = elevatorPosition.get();
    //                 piviotIO.setPosition(position);
    //             });
    // }

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
        // return run(
        //         () -> {
        //             piviotIO.setPosition(position);
        //         }, );
        return Commands.runOnce(
                () -> {
                    piviotIO.setPosition(position);
                },
                this);
    }

    public double getPosition() {
        return elevatorInputs.position;
    }
}
