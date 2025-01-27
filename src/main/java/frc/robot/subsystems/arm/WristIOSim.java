package frc.robot.subsystems.arm;

public class WristIOSim implements WristIO {
    private double position = 0;
    private double positionSetpoint = 0;

    public void updateInputs(WristIOInputs inputs) {

        final double stepAmount = 1;

        if (positionSetpoint > position) {
            position += stepAmount * 0.02;
        } else if (positionSetpoint < position) {
            position -= stepAmount * 0.02;
        }

        inputs.atTarget = true;
        inputs.position = position;
    }

    public void setPosition(double position) {
        this.positionSetpoint = position;
    }

    public void setVoltage(double voltage) {}

    public void zeroPosition() {
        this.position = 0;
    }

    public void encoderUpdate() {}

    public double getPosition() {
        return position;
    }
}
