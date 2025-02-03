package frc.robot.subsystems.arm;

public class WristIOSim implements WristIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(WristIOInputs inputs) {
        position += 0.02 * voltage;

        inputs.position = position;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
