package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class IntakeRollerIOSim implements IntakeRollerIO {
    public double voltage;
    public double speed;

    public LoggedNetworkBoolean intakeGPSensorBool =
            new LoggedNetworkBoolean("Intake Sensor Sim", true);

    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.speed = speed;
        inputs.sensor = intakeGPSensorBool.get();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
