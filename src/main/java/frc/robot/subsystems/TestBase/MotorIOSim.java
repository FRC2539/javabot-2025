package frc.robot.subsystems.TestBase;

public class MotorIOSim implements MotorIO{
    private double simSpeed = 0;
    public void updateInputs(MotorIOInputs inputs) {
        inputs.speed = simSpeed;
    }

    public void setMotorSpeed(double speeds) {
        simSpeed = speeds;
    }
}
