package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeRollerTalonFX implements IntakeRollerIO {
    private final TalonFX intakeroller = new TalonFX(13, "CANivore");

    public IntakeRollerTalonFX() {
        intakeroller.setVoltage(0);
    }

    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.speed = intakeroller.getVelocity().refresh().getValueAsDouble();
        inputs.voltage = intakeroller.getMotorVoltage().refresh().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        intakeroller.set(voltage);
    }
}
