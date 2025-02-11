package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class IntakeConstants {
    public static final int idFlipper = 17;

    public static final int openPosit = 100;

    public static final int closedPosit = 0;

    public static final int idRoller = 18;

    public static final String canbusRoller = "CANivore";

    public static final String canbusFlipper = "CANivore";

    public static final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();

    public static final SupplyCurrentLimitConfiguration rollerSupplyCurrentLimit =
            new SupplyCurrentLimitConfiguration();

    public static final int rollerContinuousCurrentLimit = 90;

    public static final int intakeGPSensor = 99;
}
