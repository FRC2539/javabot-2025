package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ArmConstants;

public class WristIONeo550 implements WristIO {
    private SparkMax wristMotor =
            new SparkMax(ArmConstants.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private PIDController pidController;
    private double reference;

    public WristIONeo550() {
        wristMotor.getEncoder().setPosition(0);

        pidController = new PIDController(ArmConstants.WRIST_KP, ArmConstants.WRIST_KI, ArmConstants.WRIST_KD);
    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.position = wristMotor.getEncoder().getPosition();
        inputs.atTarget = false; // ?
        inputs.voltage = wristMotor.getBusVoltage();
        inputs.current = wristMotor.getOutputCurrent();
        inputs.temperature = wristMotor.getMotorTemperature();
    }

    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public void setPosition(double position) {
        reference = position;

        // if (position > wristMotor.getEncoder().getPosition()) {
        //     while (position > wristMotor.getEncoder().getPosition()) {
        //         wristMotor.set(12);
        //     }
        // } else if (position < wristMotor.getEncoder().getPosition()) {
        //     while (position < wristMotor.getEncoder().getPosition()) {
        //         wristMotor.set(-12);
        //     }
        // }
    }

    public void zeroPosition() {
        wristMotor.getEncoder().setPosition(0);
    }

    public void encoderUpdate() {

        while (!pidController.atSetpoint()) {

            wristMotor.set(pidController.calculate(wristMotor.getEncoder().getPosition(), reference));
        }
    }
}
