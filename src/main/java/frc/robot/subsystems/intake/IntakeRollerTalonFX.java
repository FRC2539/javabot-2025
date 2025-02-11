package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.IntakeConstants;

public class IntakeRollerTalonFX implements IntakeRollerIO {
    private final TalonSRX intakeroller = new TalonSRX(IntakeConstants.idRoller);

    private DigitalInput intakeGPSensor = new DigitalInput(IntakeConstants.intakeGPSensor);

    public IntakeRollerTalonFX() {
        intakeroller.set(ControlMode.PercentOutput, 0);

        intakeroller.enableVoltageCompensation(true);

        // intakeroller.configSupplyCurrentLimit(IntakeConstants.rollerSupplyCurrentLimit);

        // intakeroller.configContinuousCurrentLimit(IntakeConstants.rollerContinuousCurrentLimit);

        intakeroller.setNeutralMode(NeutralMode.Brake);
    }

    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.speed = intakeroller.getSelectedSensorVelocity();
        inputs.voltage = intakeroller.getMotorOutputVoltage();
        inputs.temperature = intakeroller.getTemperature();
        inputs.current = intakeroller.getStatorCurrent();
        inputs.sensor = intakeGPSensor.get();
    }

    public void setVoltage(double voltage) {
        intakeroller.set(ControlMode.PercentOutput, voltage / 12.0);
    }
}
