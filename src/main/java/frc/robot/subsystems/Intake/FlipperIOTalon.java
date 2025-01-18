package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class FlipperIOTalon implements FlipperIO {
    private final TalonFX flipperMotor = new TalonFX(300);

    public void updateInputs(FlipperIOInputs inputs) {
        inputs.position = flipperMotor.getPosition().refresh().getValueAsDouble();
        inputs.voltage = flipperMotor.getMotorVoltage().refresh().getValueAsDouble();
    }

    public void setOpen() {
        flipperMotor.setPosition(100);
    }

    public void setClose() {
        flipperMotor.setPosition(0);
    }

    public void setVoltage(double voltagè) {
        flipperMotor.setVoltage(voltagè);
    }
}
