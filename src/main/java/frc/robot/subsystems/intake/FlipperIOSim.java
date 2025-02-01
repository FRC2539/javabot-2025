package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;

public class FlipperIOSim implements FlipperIO {

    private double position;
    private double voltage;

    public void updateInputs(FlipperIOInputs inputs) {
        inputs.position = position;
        inputs.voltage = voltage;
    }

    public void setOpen() {
        position = IntakeConstants.openPosit;
    }

    public void setClose() {
        position = IntakeConstants.closedPosit;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void resetPosition(double position) {
        this.position = position;
    }
}
