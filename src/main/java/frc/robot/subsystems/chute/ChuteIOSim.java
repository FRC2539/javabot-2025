package frc.robot.subsystems.chute;

import frc.robot.constants.ChuteConstants;

public class ChuteIOSim implements ChuteIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ChuteIOInputs inputs) {
        position += 0.02 * voltage * 20;

        inputs.voltage = voltage;

        if (position > ChuteConstants.upperLimit && voltage > 0) {
            position = ChuteConstants.upperLimit;
            inputs.current = 30;
        } else if (position < ChuteConstants.lowerLimit  && voltage < 0) {
            position = ChuteConstants.lowerLimit;
            inputs.current = 30;
        } else {
            inputs.current = 0;
        }
        inputs.position = position;
        // inputs.throughboreEncoderPosition = position;
        // inputs.throughboreConnected = true;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {}
}
