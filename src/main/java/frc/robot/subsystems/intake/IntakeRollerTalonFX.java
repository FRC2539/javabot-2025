package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.IntakeConstants;

public class IntakeRollerTalonFX implements IntakeRollerIO {
    private final TalonFX intakeroller = new TalonFX(IntakeConstants.idRoller, IntakeConstants.canbusRoller);

    public IntakeRollerTalonFX() {
        intakeroller.setVoltage(0);

        intakeroller.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(IntakeConstants.currentLimit));

        intakeroller.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.speed = intakeroller.getVelocity().getValueAsDouble();
        inputs.voltage = intakeroller.getMotorVoltage().getValueAsDouble();
        inputs.temperature = intakeroller.getDeviceTemp().getValueAsDouble();
        inputs.current = intakeroller.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        intakeroller.set(voltage);
    }
}
