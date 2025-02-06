package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    private ElevatorIO piviotIO;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private SysIdRoutine elevatorSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            state ->
                                    SignalLogger.writeString(
                                            "SysIdElevator_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (voltage) -> piviotIO.setVoltage(voltage.in(Volts)), null, this));

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.piviotIO = elevatorIO;
        setDefaultCommand(setPosition(0));
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
