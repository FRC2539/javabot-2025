package frc.robot.subsystems.chute;

public class ChuteIOSim implements ChuteIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ChuteIOInputs inputs) {
        position += 0.02 * voltage;

        inputs.position = position;
        // inputs.throughboreEncoderPosition = position;
        // inputs.throughboreConnected = true;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
