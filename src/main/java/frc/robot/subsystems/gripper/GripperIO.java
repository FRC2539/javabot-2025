package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
    public void updateInputs(GripperIOInputs inputs);

    @AutoLog
    public class GripperIOInputs {
        public double speedLeft = 0;
        public double voltageLeft = 0;
        public double temperatureLeft = 0;
        public double currentLeft = 0;

        public double speedRight = 0;
        public double voltageRight = 0;
        public double temperatureRight = 0;
        public double currentRight = 0;

        public boolean firstSensor = false;

        public boolean secondSensor = false;
    }

    public void setVoltage(double voltage);

    public void setVoltageLeft(double voltage);

    public void setVoltageRight(double voltage);
}
