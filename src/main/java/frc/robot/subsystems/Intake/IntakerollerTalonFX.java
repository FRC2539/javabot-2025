package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakerollerTalonFX implements IntakerollerIO {
    private final TalonFX intakeroller = new TalonFX(45, "CANivore");

    public IntakerollerTalonFX() {
        intakeroller.setVoltage(0);
    }

    public void updateInputs(IntakerollerIOInputs inputs) {
        inputs.speed = intakeroller.getVelocity().refresh().getValueAsDouble();
        inputs.voltage = intakeroller.getVelocity().refresh().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        intakeroller.set(voltage);
    }
}
