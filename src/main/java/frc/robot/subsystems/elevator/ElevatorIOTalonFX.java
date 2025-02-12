package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
    // TBD: Hardcode IDs or add support to make changeable in method
    private final TalonFX elevatorLeader =
            new TalonFX(ElevatorConstants.elevatorLeaderId, ElevatorConstants.elevatorLeaderCanbus);
    // private final TalonFX elevatorFollower =
    //         new TalonFX(
    //                 ElevatorConstants.elevatorFollowerId,
    // ElevatorConstants.elevatorFollowerCanbus);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    // Status signals for efficient updates
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Current> currentSignal;

    public ElevatorIOTalonFX() {
        elevatorLeader.setPosition(0);

        motionMagicVoltage.Slot = 0;

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ElevatorConstants.upperLimit)
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ElevatorConstants.lowerLimit);

        TalonFXConfiguration config =
                new TalonFXConfiguration()
                        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                        .withSlot0(ElevatorConstants.slot0Configs)
                        .withMotionMagic(ElevatorConstants.motionMagicConfigs)
                        .withCurrentLimits(ElevatorConstants.currentLimit);

        elevatorLeader.getConfigurator().apply(config);
        // elevatorFollower.getConfigurator().apply(config);

        // elevatorFollower.setControl(new Follower(elevatorFollower.getDeviceID(), false));

        elevatorLeader.setNeutralMode(NeutralModeValue.Brake);
        // elevatorFollower.setNeutralMode(NeutralModeValue.Brake);

        // Initialize status signals
        positionSignal = elevatorLeader.getPosition();
        voltageSignal = elevatorLeader.getMotorVoltage();
        velocitySignal = elevatorLeader.getVelocity();
        temperatureSignal = elevatorLeader.getDeviceTemp();
        currentSignal = elevatorLeader.getStatorCurrent();

        // Set update frequency and optimize bus utilization
        PhoenixUtil.tryUntilOk(
                5,
                () ->
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                positionSignal,
                                voltageSignal,
                                velocitySignal,
                                temperatureSignal,
                                currentSignal));

        PhoenixUtil.tryUntilOk(
                5, () -> ParentDevice.optimizeBusUtilizationForAll(0, elevatorLeader));
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        // Refresh all signals at once
        BaseStatusSignal.refreshAll(
                positionSignal, voltageSignal, velocitySignal, temperatureSignal, currentSignal);

        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.speed = velocitySignal.getValueAsDouble();
        inputs.temperature = temperatureSignal.getValueAsDouble();
        inputs.current = currentSignal.getValueAsDouble();
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
