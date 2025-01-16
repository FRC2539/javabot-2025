package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmIOTalonFX implements ArmIO {
    private TalonFX armMotor = new TalonFX(50);
    private TalonFX wristMotor = new TalonFX(53);

    public ArmIOTalonFX() {
        armMotor.setPosition(0);
        wristMotor.setPosition(0);
    }

    public void updateInputs(ArmIOInputs inputs) {

        inputs.position = armMotor.getPosition().refresh().getValueAsDouble();
        inputs.voltage = armMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = armMotor.getVelocity().refresh().getValueAsDouble();

        inputs.position = wristMotor.getPosition().refresh().getValueAsDouble();
        inputs.voltage = wristMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = wristMotor.getVelocity().refresh().getValueAsDouble();
    }

    public void setArmVoltage(double voltage) {
        armMotor.setVoltage(voltage);
    }

    public void setArmPosition(double position) {
        armMotor.setPosition(position);
    }

    public void setWristVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public void setWristPosition(double position) {
        wristMotor.setPosition(position);
    }
}
