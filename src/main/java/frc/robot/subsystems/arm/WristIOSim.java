package frc.robot.subsystems.arm;

public class WristIOSim implements WristIO {
    private double position = 0;

    public void updateInputs(WristIOInputs inputs) {
        inputs.atTarget = true;
        inputs.position = position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public void setVoltage(double voltage) {}

    public void zeroPosition() {
        this.position = 0;
    }
}
