package frc.robot.subsystems.chute;

import frc.robot.constants.ChuteConstants;

public class ChuteIOSim implements ChuteIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ChuteIOInputs inputs) {
        position += 0.02 * voltage;

        inputs.position = position;

        if (position > ChuteConstants.upperLimit) {
            position = ChuteConstants.upperLimit;
            inputs.current = 30;
        } else if (position < ChuteConstants.lowerLimit) {
            position = ChuteConstants.lowerLimit;
            inputs.current = 30;
        } else {
            inputs.current = 0;
        }
        // inputs.throughboreEncoderPosition = position;
        // inputs.throughboreConnected = true;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {}
}
