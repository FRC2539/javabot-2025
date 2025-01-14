package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSim implements ElevatorIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = position;
        inputs.voltage = voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {

        this.position = position;
    }

    public void encoderUpdate() {}

    public PIDController getPIDController() {
        return null;
    }
}
