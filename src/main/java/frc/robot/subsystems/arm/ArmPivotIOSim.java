package frc.robot.subsystems.arm;

public class ArmPivotIOSim implements ArmPivotIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ArmPivotIOInputs inputs) {
        inputs.position = position;
        inputs.voltage = voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setPosition(double position) {
        this.position = position;
    }
}
