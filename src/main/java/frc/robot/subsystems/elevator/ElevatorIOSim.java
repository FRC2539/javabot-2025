package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSim implements ElevatorIO {
    private double position = 0;
    private double positionSetpoint = 0;
    private double voltage = 0;
    private PIDController sussyAhh = new PIDController(5, 0, 0);

    private boolean positionControl = false;

    public void updateInputs(ElevatorIOInputs inputs) {
        if (positionControl) {
            voltage = sussyAhh.calculate(position, positionSetpoint);
        }
        // position += voltage * 0.02;

        inputs.position = positionSetpoint;
        inputs.voltage = voltage;
    }

    public void setVoltage(double voltage) {
        positionControl = false;
        this.voltage = voltage;
    }

    public void setPosition(double position) {
        positionControl = true;
        this.positionSetpoint = position;
    }

    public void resetPosition(double position) {
        this.position = position;
    }
}
