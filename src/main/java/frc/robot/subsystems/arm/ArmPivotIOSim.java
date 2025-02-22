package frc.robot.subsystems.arm;

public class ArmPivotIOSim implements ArmPivotIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ArmPivotIOInputs inputs) {
        position += 0.02 * voltage * 0.5;

        inputs.position = position;
        inputs.throughboreEncoderPosition = position;
        inputs.throughboreConnected = true;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
