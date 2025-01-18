package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    // TBD: Hardcode IDs or add support to make changeable in method
    private final TalonFX elevatorLeader = new TalonFX(41, "CANivore");
    private final TalonFX elevatorFollower = new TalonFX(42, "CANivore");
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ElevatorIOTalonFX() {
        elevatorLeader.setPosition(0);

        motionMagicVoltage.Slot = 0;

        TalonFXConfigurator talonConfig = elevatorLeader.getConfigurator();

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();

        talonConfig.apply(
                new TalonFXConfiguration()
                        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                        .withSlot0(ElevatorConstants.slot0Configs)
                        .withMotionMagic(ElevatorConstants.motionMagicConfigs));

        elevatorFollower.setControl(new Follower(elevatorFollower.getDeviceID(), false));
    }

    public void updateInputs(ElevatorIOInputs inputs) {

        inputs.position = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeader.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = elevatorLeader.getVelocity().refresh().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        elevatorLeader.setVoltage(voltage);
    }

    public void setPosition(double position) {
        elevatorLeader.setControl(motionMagicVoltage.withPosition(position));
    }
}
