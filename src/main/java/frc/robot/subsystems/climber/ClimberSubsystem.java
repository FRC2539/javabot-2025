package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

    private ClimberIO piviotIO;
    private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO climberIO) {
        this.piviotIO = climberIO;
        setDefaultCommand(stop());
    }

    public void periodic() {

        piviotIO.updateInputs(climberInputs);

        Logger.processInputs("RealOutputs/Climber", climberInputs);

        // if (climberInputs.voltage < 0 && climberInputs.position <= lowerLimit) {
        //     this.piviotIO.setVoltage(0);
        // }

        // if (climberInputs.voltage > 0 && climberInputs.position >= upperLimit) {
        //     this.piviotIO.setVoltage(0);
        // }
    }

    public Command zeroClimberCommand() {
        return run(
                () -> {
                    piviotIO.setPosition(0);
                });
    }

    public Command moveClimberUpVoltage() {
        return setVoltage(12);
    }

    public Command moveClimberDownVoltage() {
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
        return climberInputs.position;
    }

    public Command upPosition() {
        return run(
                () -> {
                    piviotIO.setPosition(ClimberConstants.upperLimit);
                });
    }

    public Command downPosition() {
        return run(
                () -> {
                    piviotIO.setPosition(ClimberConstants.lowerLimit);
                });
    }

    public Command stop() {
        return run(
                () -> {
                    piviotIO.setVoltage(0);
                });
    }
}
