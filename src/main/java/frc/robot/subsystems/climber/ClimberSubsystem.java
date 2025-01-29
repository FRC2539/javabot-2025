package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

    private ClimberIO piviotIO;
    private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    private final double lowerLimit = 0;
    private final double upperLimit = 100;

    public ClimberSubsystem(ClimberIO climberIO) {
        this.piviotIO = climberIO;
        setDefaultCommand(setPosition(0));
    }

    public void periodic() {

        piviotIO.updateInputs(climberInputs);

        Logger.processInputs("RealOutputs/Climber", climberInputs);

        if (climberInputs.voltage < 0 && climberInputs.position <= lowerLimit) {
            this.piviotIO.setVoltage(0);
        }

        if (climberInputs.voltage > 0 && climberInputs.position >= upperLimit) {
            this.piviotIO.setVoltage(0);
        }
    }

    public Command zeroClimberCommand() {
        return run(
                () -> {
                    piviotIO.setPosition(0);
                });
    }

    public Command moveClimberUp() {
        return setVoltage(12).until(() -> climberInputs.position >= upperLimit);
    }

    public Command moveClimberDown() {
        return setVoltage(-12).until(() -> climberInputs.position <= lowerLimit);
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
