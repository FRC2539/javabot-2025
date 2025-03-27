package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class ArmConstants {

    public static final int ARM_PIVOT_MOTOR_ID = 10;
    public static final String ARM_PIVOT_CANBUS = "rio";

    public static final int ARM_THROUGHBORE_ENCODER_ID = 1;

    //     public static final Slot0Configs slot0Configs =
    //             new Slot0Configs()
    //                     .withKA(0)
    //                     .withKD(0)
    //                     .withKG(0)
    //                     .withKI(0)
    //                     .withKP(0)
    //                     .withKS(0)
    //                     .withKV(0)
    //                     .withGravityType(GravityTypeValue.Elevator_Static);
    //     ;
    //     public static final MotionMagicConfigs motionMagicConfigs =
    //             new MotionMagicConfigs()[]\

    //                     .withMotionMagicAcceleration(4) // these are guesses, come back here
    //                     .withMotionMagicCruiseVelocity(4) // also guess
    //                     .withMotionMagicJerk(4);

    public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    public static final double ARM_KP = 8.5; // 5 3.5 5
    public static final double ARM_KD = 0.000; // 0
    public static final double ARM_KI = 0.0; // .1 .25

    public static final double ARM_TOLERANCE = 0.0436; // 0.03

    public static final double upperLimit = 2.3;
    public static final double lowerLimit = -0.5;
}
