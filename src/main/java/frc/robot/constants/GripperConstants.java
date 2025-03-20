package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class GripperConstants {
    public static final int leftMotorId = 11;
    public static final int rightMotorId = 12;

    public static final CurrentLimitsConfigs currentLimit =
            new CurrentLimitsConfigs().withStatorCurrentLimit(50);

    public static final String canbus = "rio";

    public static final double handoffVoltage = 3; // 4.8
    public static final double slowHandoffVoltage = 1.8; // 2.2
    public static final double placeVoltage = -6;
    public static final double downvoltage = -1;
    public static final double upvoltage = 1;

    public static final int initialSensorChannel = 0;

    public static final int secondSensorChannel = 0;
}
