package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.IntakeConstants;

public class FlipperIOTalon implements FlipperIO {
    private final TalonFX flipperMotor = new TalonFX(IntakeConstants.idFlipper);
    private final PositionVoltage openPV = new PositionVoltage(IntakeConstants.openPosit);
    private final PositionVoltage closedPV = new PositionVoltage(IntakeConstants.closedPosit);

    public FlipperIOTalon() {

        flipperMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(IntakeConstants.currentLimit));

        flipperMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(FlipperIOInputs inputs) {
        inputs.position = flipperMotor.getPosition().getValueAsDouble();
        inputs.voltage = flipperMotor.getMotorVoltage().getValueAsDouble();
        inputs.temperature = flipperMotor.getDeviceTemp().getValueAsDouble();
        inputs.current = flipperMotor.getStatorCurrent().getValueAsDouble();
    } 

    public void setOpen() {

        flipperMotor.setControl(openPV);
    }

    public void setClose() {
        flipperMotor.setControl(closedPV);
    }

    public void setVoltage(double voltage) {
        flipperMotor.setVoltage(voltage);
    }
}
