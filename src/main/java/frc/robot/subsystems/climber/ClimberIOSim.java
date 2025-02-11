package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {
    private double position = 0;
    private double positionSetpoint = 0;
    private double voltage = 0;

    public void updateInputs(ClimberIOInputs inputs) {
        final double stepAmount = 1;
        if (positionSetpoint > position) {
            position += stepAmount * 0.02;
        } else if (positionSetpoint < position) {
            position -= stepAmount * 0.02;
        }

        inputs.position = position;
        inputs.voltage = voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {
        this.positionSetpoint = position;
    }

    public void resetPosition(double position) {
        this.position = position;
    }
}
