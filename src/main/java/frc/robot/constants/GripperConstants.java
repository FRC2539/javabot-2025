package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class GripperConstants {
    public static final int leftMotorId = 11;
    public static final int rightMotorId = 12;

    public static final CurrentLimitsConfigs currentLimit =
            new CurrentLimitsConfigs().withStatorCurrentLimit(50);

    public static final String canbus = "rio";

    public static final double handoffVoltage = 5.55;
    public static final double placeVoltage = -4.3;
    public static final double downvoltage = -1;
    public static final double upvoltage = 1;

    public static final int pieceSensorChannel = 0;
}
