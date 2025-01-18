package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlipperIOTalon implements FlipperIO {
    private final TalonFX flipperMotor = new TalonFX(300);
    private final PositionVoltage openPV= new PositionVoltage(1);
    private final PositionVoltage closedPV = new PositionVoltage(0);

    public void updateInputs(FlipperIOInputs inputs) {
        inputs.position = flipperMotor.getPosition().refresh().getValueAsDouble();
        inputs.voltage = flipperMotor.getMotorVoltage().refresh().getValueAsDouble();
    }

    public void setOpen() {
        
        flipperMotor.setControl(openPV);
    }

    public void setClose() {
        flipperMotor.setControl(closedPV);
    }

    public void setVoltage(double voltagè) {
        flipperMotor.setVoltage(voltagè);
    }
}
