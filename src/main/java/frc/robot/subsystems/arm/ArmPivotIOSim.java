package frc.robot.subsystems.arm;

public class ArmPivotIOSim implements ArmPivotIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ArmPivotIOInputs inputs) {
        position += 0.02 * voltage;

        inputs.position = position;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
