package frc.robot.subsystems.arm;

public class ArmPivotIOSim implements ArmPivotIO {
    private double position = 0;
    private double voltage = 0;
    private double positionSetpoint = 0;

    public void updateInputs(ArmPivotIOInputs inputs) {
        final double stepAmount = 1;
        if (positionSetpoint > position) {
            position += stepAmount * 0.02;
        } else if (positionSetpoint < position) {
            position -= stepAmount * 0.02;
        }

        inputs.position = position;
        inputs.voltage = voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {
        this.positionSetpoint = position;
    }
}
