package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ClimberConstants {
    public static final Slot0Configs slot0Configs =
            new Slot0Configs()
                    .withKA(0)
                    .withKD(0)
                    .withKG(0)
                    .withKI(0)
                    .withKP(0)
                    .withKS(0)
                    .withKV(0);
    ;
    public static final MotionMagicConfigs motionMagicConfigs =
            new MotionMagicConfigs()
                    .withMotionMagicAcceleration(4) // these are guesses, come back here
                    .withMotionMagicCruiseVelocity(4) // also guess
                    .withMotionMagicJerk(4);

    public static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    public static final double upperLimit = 100;

    public static final double lowerLimit = 0;

    public static final int CLIMBER_ID = 16;

    public static final int CLIMBER_HEAD_ID = 40;

    public static final String canbus = "rio";

    public static final String WinchCanbus = "CANivore";

    public static final double ClimberHeadCurrent = 30;
}
