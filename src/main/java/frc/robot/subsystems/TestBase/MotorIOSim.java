package frc.robot.subsystems.TestBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class MotorIOSim implements MotorIO{
    LoggedNetworkNumber simSpeed;

    public MotorIOSim(String name) {
        simSpeed = new LoggedNetworkNumber(name, 1);
    }
    
    public void updateInputs(MotorIOInputs inputs) {
        inputs.speed = simSpeed.get();
    }

    public void setMotorSpeed(double speeds) {
        simSpeed.set(speeds);
    }

    public double getMotorSpeed(){
        return simSpeed.get();
    }
}
