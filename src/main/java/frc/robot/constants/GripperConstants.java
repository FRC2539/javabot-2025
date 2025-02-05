package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class GripperConstants {
    public static final int id = 12;

    public static final CurrentLimitsConfigs currentLimit =
            new CurrentLimitsConfigs().withStatorCurrentLimit(50);

    public static final String canbus = "CANivore";

    public static final int gripperSensorChannel = 60;
}
