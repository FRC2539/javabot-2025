package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    // TBD: Hardcode IDs or add support to make changeable in method
    private final TalonFX elevatorLeader =
            new TalonFX(ElevatorConstants.elevatorLeaderId, ElevatorConstants.elevatorLeaderCanbus);
    // private final TalonFX elevatorFollower =
    //         new TalonFX(
    //                 ElevatorConstants.elevatorFollowerId,
    // ElevatorConstants.elevatorFollowerCanbus);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ElevatorIOTalonFX() {
        elevatorLeader.setPosition(0);

        motionMagicVoltage.Slot = 0;

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ElevatorConstants.upperLimit)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ElevatorConstants.lowerLimit);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(0.56250);

        TalonFXConfiguration config =
                new TalonFXConfiguration()
                        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                        .withSlot0(ElevatorConstants.slot0Configs)
                        .withMotionMagic(ElevatorConstants.motionMagicConfigs)
                        .withCurrentLimits(ElevatorConstants.currentLimit)
                        .withFeedback(feedbackConfigs);

        elevatorLeader.getConfigurator().apply(config);
        // elevatorFollower.getConfigurator().apply(config);

        // elevatorFollower.setControl(new Follower(elevatorFollower.getDeviceID(), false));

        elevatorLeader.setNeutralMode(NeutralModeValue.Brake);
        // elevatorFollower.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(ElevatorIOInputs inputs) {

        inputs.position = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeader.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = elevatorLeader.getVelocity().refresh().getValueAsDouble();
        inputs.temperature = elevatorLeader.getDeviceTemp().getValueAsDouble();
        inputs.current = elevatorLeader.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        elevatorLeader.setVoltage(voltage);
    }

    public void setPosition(double position) {
        if (position > ElevatorConstants.upperLimit) {
            position = ElevatorConstants.upperLimit;
        }
        if (position < ElevatorConstants.lowerLimit) {
            position = ElevatorConstants.lowerLimit;
        }
        elevatorLeader.setControl(motionMagicVoltage.withPosition(position));
    }

    public void resetPosition(double position) {
        elevatorLeader.setPosition(position);
    }
}
