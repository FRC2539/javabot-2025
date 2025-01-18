package frc.robot.subsystems.intake;

public class FlipperIOSim implements FlipperIO {

    private double position;

    public void updateInputs(FlipperIOInputs inputs) {
        inputs.position = position;
        inputs.voltage = 0;
    }

    public void setOpen() {
        position = 100;
    }

    public void setClose() {
        position = 0;
    }

    public void setVoltage(double voltage) {}
}
