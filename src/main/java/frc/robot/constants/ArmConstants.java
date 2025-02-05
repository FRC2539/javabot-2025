package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class ArmConstants {

    public static final int ARM_PIVOT_MOTOR_ID = 0; // not correct motor ID
    public static final String ARM_PIVOT_CANBUS = "CANivore";

    public static final int ARM_THROUGHBORE_ENCODER_ID = 0;

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
    //             new MotionMagicConfigs()
    //                     .withMotionMagicAcceleration(4) // these are guesses, come back here
    //                     .withMotionMagicCruiseVelocity(4) // also guess
    //                     .withMotionMagicJerk(4);

    public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();

    public static final double ARM_KP = 0;
    public static final double ARM_KD = 0;
    public static final double ARM_KI = 0;

    public static final double ARM_TOLERANCE = 0.1;

    public static final double upperLimit = 100;
    public static final double lowerLimit = 0;
}
