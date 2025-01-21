package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ArmConstants;

public class WristIONeo550 implements WristIO {
    private SparkMax SparkMax =
            new SparkMax(ArmConstants.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private PIDController pidController;
    private double reference;

    public WristIONeo550(double kP, double kI, double kD) {
        SparkMax.getEncoder().setPosition(0);

        pidController = new PIDController(kP, kI, kD);
    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.position = SparkMax.getEncoder().getPosition();
        inputs.atTarget = false; // ?
        inputs.voltage = SparkMax.getBusVoltage();
        inputs.current = SparkMax.getOutputCurrent();
        inputs.temperature = SparkMax.getMotorTemperature();
    }

    public void setVoltage(double voltage) {
        SparkMax.setVoltage(voltage);
    }

    public void setPosition(double position) {
        reference = position;
        if (position > SparkMax.getEncoder().getPosition()) {
            while (position > SparkMax.getEncoder().getPosition()) {
                SparkMax.set(12);
            }
        } else if (position < SparkMax.getEncoder().getPosition()) {
            while (position < SparkMax.getEncoder().getPosition()) {
                SparkMax.set(-12);
            }
        }
    }

    public void zeroPosition() {
        SparkMax.getEncoder().setPosition(0);
    }

    public void encoderUpdate() {

        while (!pidController.atSetpoint()) {

            SparkMax.set(pidController.calculate(SparkMax.getEncoder().getPosition(), reference));
        }
    }
}
