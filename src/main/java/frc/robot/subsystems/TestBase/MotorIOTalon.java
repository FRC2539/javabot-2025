package frc.robot.subsystems.TestBase;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.TestBase.MotorIO.MotorIOInputs;

public class MotorIOTalon implements MotorIO {
    


    private TalonFX motor;
    public MotorIOTalon(int id) {
        motor = new TalonFX(id);
    }

    public void updateInputs(MotorIOInputs inputs) {
        inputs.speed = motor.get();
        inputs.voltage = motor.getMotorVoltage().refresh().getValueAsDouble();
    }

    public void setMotorSpeed(double speeds) {
        motor.set(speeds);
    }

    public double getMotorSpeed(){
        return motor.get();
    }
}

