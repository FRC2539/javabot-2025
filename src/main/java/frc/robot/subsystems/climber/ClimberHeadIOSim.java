package frc.robot.subsystems.climber;

public class ClimberHeadIOSim implements ClimberHeadIO {
    private double voltage = 0;
    private double position = 0;

    public void updateInputs(ClimberHeadIOInputs inputs) {

        position += 0.02 * voltage;

        inputs.position = position;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
