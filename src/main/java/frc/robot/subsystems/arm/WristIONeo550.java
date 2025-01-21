package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.ArmConstants;

public class WristIONeo550 implements WristIO {
    private SparkMax SparkMax =
            new SparkMax(ArmConstants.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkClosedLoopController pidController;

    public WristIONeo550() {
        SparkMax.getEncoder().setPosition(0);

        pidController = SparkMax.getClosedLoopController();
        pidController.setReference(100, ControlType.kPosition); // what??
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
        SparkMax.getEncoder().setPosition(position);
    }

    public void zeroPosition() {
        SparkMax.getEncoder().setPosition(0);
    }
}
