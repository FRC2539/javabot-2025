package frc.robot.subsystems.kicker;

public class KickerIOSim implements KickerIO {
    public double voltage;
    public double position;

    public void updateInputs(KickerIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.position = position;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {
        this.position = position;
    }
}
