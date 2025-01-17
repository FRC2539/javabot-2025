package frc.robot.subsystems.Arm;

import frc.robot.subsystems.Arm.ArmrollerIO.ArmrollerIOInputs;

public class ArmrollerIOSim {
    public double voltage;
    public double speed;

    public void updateInputs(ArmrollerIOInputs inputs) {
        voltage = inputs.voltage;
        speed = inputs.speed;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
