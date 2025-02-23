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
        public boolean sensorLeft = false;
        public double throughboreEncoderPositionLeft = 0;
        public boolean shutdownLeft;
        public boolean throughboreConnectedLeft = false;
        public double positionLeft;

        public double speedRight = 0;
        public double voltageRight = 0;
        public double temperatureRight = 0;
        public double currentRight = 0;
        public boolean sensorRight = false;
        public double throughboreEncoderPositionRight = 0;
        public boolean shutdownRight;
        public boolean throughboreConnectedRight = false;
        public double positionRight;
    }

    public void setVoltage(double voltage);

    public void setVoltageLeft(double voltage);

    public void setVoltageRight(double voltage);
}
