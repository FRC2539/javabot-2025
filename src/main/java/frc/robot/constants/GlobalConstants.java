package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

public class GlobalConstants {
    
    public static double MAX_TRANSLATIONAL_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MAX_ROTATIONAL_SPEED = Math.PI;

    public static class ControllerConstants {
        public static final int LEFT_DRIVE_CONTROLLER = 0;
        public static final int RIGHT_DRIVE_CONTROLLER = 1;
        public static final int OPERATOR_CONTROLLER = 2;
    }
}
