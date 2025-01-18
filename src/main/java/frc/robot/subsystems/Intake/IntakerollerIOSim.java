package frc.robot.subsystems.Intake;

import frc.robot.subsystems.Intake.IntakerollerIO.IntakerollerIOInputs;

public class IntakerollerIOSim {
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
