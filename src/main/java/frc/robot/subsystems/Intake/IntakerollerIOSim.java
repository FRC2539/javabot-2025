package frc.robot.subsystems.Intake;

public class IntakerollerIOSim implements IntakerollerIO {
    public double voltage;
    public double speed;

    public void updateInputs(IntakerollerIOInputs inputs) {
        voltage = inputs.voltage;
        speed = inputs.speed;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
