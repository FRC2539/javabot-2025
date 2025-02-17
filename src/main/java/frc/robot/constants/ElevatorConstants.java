package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElevatorConstants {
    public static final Slot0Configs slot0Configs =
            new Slot0Configs()
                    .withKA(0)
                    .withKD(0)
                    .withKG(0)
                    .withKI(0)
                    .withKP(0)
                    .withKS(0)
                    .withKV(0)
                    .withGravityType(GravityTypeValue.Elevator_Static);
    ;
    public static final MotionMagicConfigs motionMagicConfigs =
            new MotionMagicConfigs()
                    .withMotionMagicAcceleration(4) // these are guesses, come back here
                    .withMotionMagicCruiseVelocity(4) // also guess
                    .withMotionMagicJerk(4);

    public static final int elevatorLeaderId = 9;

    public static final String elevatorLeaderCanbus = "CANivore";

    // public static final String elevatorFollowerCanbus = "CANivore";

    public static final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();

    public static final double lowerLimit = 10;
    public static final double upperLimit = 298.5;
}
