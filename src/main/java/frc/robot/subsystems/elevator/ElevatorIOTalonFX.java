package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Pound;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOTalonFX implements ElevatorIO {
    // TBD: Hardcode IDs or add support to make changeable in method
    private final TalonFX elevatorLeader = new TalonFX(41, "CANivore");
    private final TalonFX elevatorFollower = new TalonFX(42, "CANivore");
    public final PIDController pidController;
    private final ElevatorFeedforward elevatorFeedforward;
    private double targetHeight;

    public final void follower() {
        elevatorFollower.setControl(new Follower(42, false));
    }

    public ElevatorIOTalonFX(PIDController pidController, ElevatorFeedforward elevatorFeedforward) {
        this.pidController = pidController;
        this.elevatorFeedforward = elevatorFeedforward;
        elevatorLeader.setPosition(0);
        follower();
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        encoderUpdate();

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
            double currentPosition = elevatorLeader
                                            .getPosition()
                                            .getValueAsDouble();
            elevatorLeader.set(
                    pidController.calculate(
                                    currentPosition /*this is the encoder position*/,
                                    targetHeight)
                            + elevatorFeedforward.calculate(targetHeight - currentPosition));
        }
    }
}
