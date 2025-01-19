package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX gripperRoller = new TalonFX(12);

    private final StatusSignal<AngularVelocity> velocitySupplier = gripperRoller.getVelocity();
    private final StatusSignal<Voltage> voltageSupplier = gripperRoller.getMotorVoltage();
    private final StatusSignal<Current> currentSupplier = gripperRoller.getStatorCurrent();
    private final StatusSignal<Temperature> temperatureSupplier = gripperRoller.getDeviceTemp();

    public GripperIOFalcon() {
        gripperRoller.setPosition(0);
    }

    public void updateInputs(GripperIOInputs inputs) {
        StatusSignal.refreshAll(
                velocitySupplier, voltageSupplier, currentSupplier, temperatureSupplier);

        inputs.voltage = voltageSupplier.getValueAsDouble();
        inputs.speed = velocitySupplier.getValueAsDouble();
        inputs.current = currentSupplier.getValueAsDouble();
        inputs.temperature = temperatureSupplier.getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        gripperRoller.setVoltage(voltage);
    }
}
