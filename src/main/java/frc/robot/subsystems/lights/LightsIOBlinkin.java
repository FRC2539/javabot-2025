package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class LightsIOBlinkin implements LightsIO {
    PWMSparkMax blinkinPWMController;

    public LightsIOBlinkin(int port) {
        blinkinPWMController = new PWMSparkMax(port);
    }

    public void setPattern(Pattern pattern) {
        blinkinPWMController.set(pattern.PWMValue);
    }
}