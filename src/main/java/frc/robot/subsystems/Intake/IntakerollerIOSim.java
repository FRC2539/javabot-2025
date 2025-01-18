package frc.robot.subsystems.Intake;

public class IntakerollerIOSim {
    public double voltage;
    public double speed;

    public void updateInputs(IntakerollerIOSim inputs) {
        voltage = inputs.voltage;
        speed = inputs.speed;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
