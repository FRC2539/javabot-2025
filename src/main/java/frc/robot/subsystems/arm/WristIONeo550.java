package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;

public class WristIONeo550 implements WristIO {
    private SparkMax wristMotor =
            new SparkMax(ArmConstants.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder throughboreEncoder =
            new DutyCycleEncoder(ArmConstants.WRIST_THROUGHBORE_ENCODER_ID, 2 * Math.PI, 0);

    public WristIONeo550() {
        wristMotor.getEncoder().setPosition(0);
    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.position = wristMotor.getEncoder().getPosition();
        inputs.voltage = wristMotor.getBusVoltage() * wristMotor.getAppliedOutput();
        inputs.current = wristMotor.getOutputCurrent();
        inputs.temperature = wristMotor.getMotorTemperature();
        inputs.throughboreEncoderPosition = throughboreEncoder.get();
    }

    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }
}
