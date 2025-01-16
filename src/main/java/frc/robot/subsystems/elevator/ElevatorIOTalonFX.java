package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.jni.HardwareJNI;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOTalonFX implements ElevatorIO {
    // TBD: Hardcode IDs or add support to make changeable in method
    private final TalonFX elevatorLeader = new TalonFX(41, "CANivore");
    private final TalonFX elevatorFollower = new TalonFX(42, "CANivore");
    public final PIDController pidController;
    private final ElevatorFeedforward elevatorFeedforward;
    private double targetHeight;
    private final Slot0Configs slot0Configs = new Slot0Configs();

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ElevatorIOTalonFX(PIDController pidController, ElevatorFeedforward elevatorFeedforward) {
        this.pidController = pidController;
        this.elevatorFeedforward = elevatorFeedforward;
        elevatorLeader.setPosition(0);
        

        motionMagicVoltage.Slot = 0;

        TalonFXConfigurator talonConfig = elevatorLeader.getConfigurator();
        
        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();

        final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicAcceleration(4); //these are guesses, come back here
        motionMagicConfigs.withMotionMagicCruiseVelocity(4); //also guess
        motionMagicConfigs.withMotionMagicJerk(4);
        // motionMagicConfigs.with

        slot0Configs.withKA(elevatorFeedforward.getKa()).withKD(pidController.getD()).withKG(elevatorFeedforward.getKg()).withKI(pidController.getI()).withKP(pidController.getP()).withKS(elevatorFeedforward.getKs()).withKV(elevatorFeedforward.getKv()).withGravityType(GravityTypeValue.Elevator_Static);

        talonConfig.apply(slot0Configs);
        talonConfig.apply(motionMagicConfigs);

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
        targetHeight = position;
    }

    private void encoderUpdate() {

        while (!pidController.atSetpoint()) {
            double currentPosition = elevatorLeader.getPosition().getValueAsDouble();
            elevatorLeader.set(
                    pidController.calculate(
                                    currentPosition /*this is the encoder position*/, targetHeight)
                            + elevatorFeedforward.calculate(targetHeight - currentPosition));
        }
    }
}
