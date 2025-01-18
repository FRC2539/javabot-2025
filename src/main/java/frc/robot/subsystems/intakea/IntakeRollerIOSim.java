package frc.robot.subsystems.intakea;

public class IntakeRollerIOSim implements IntakeRollerIO {
    public double voltage;
    public double speed;

    public void updateInputs(IntakeRollerIOInputs inputs) {
        voltage = inputs.voltage;
        speed = inputs.speed;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
