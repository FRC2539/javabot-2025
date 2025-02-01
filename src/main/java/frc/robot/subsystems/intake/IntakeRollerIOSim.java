package frc.robot.subsystems.intake;

public class IntakeRollerIOSim implements IntakeRollerIO {
    public double voltage;
    public double speed;

    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.speed = speed;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
