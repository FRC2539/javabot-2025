package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.GripperConstants;
import frc.robot.util.PhoenixUtil;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX armRoller = new TalonFX(GripperConstants.id, GripperConstants.canbus);

    private DigitalInput gripperGPSensor = new DigitalInput(GripperConstants.gripperSensorChannel);

    // Status signals for efficient updates
    private final StatusSignal<AngularVelocity> speedSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    public GripperIOFalcon() {
        armRoller.setPosition(0);

        armRoller
                .getConfigurator()
                .apply(new TalonFXConfiguration().withCurrentLimits(GripperConstants.currentLimit));

        armRoller.setNeutralMode(NeutralModeValue.Brake);

        // Initialize status signals
        speedSignal = armRoller.getVelocity();
        voltageSignal = armRoller.getMotorVoltage();
        currentSignal = armRoller.getStatorCurrent();
        temperatureSignal = armRoller.getDeviceTemp();

        // Set update frequency and optimize bus utilization
        PhoenixUtil.tryUntilOk(
                5,
                () ->
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                speedSignal,
                                voltageSignal,
                                currentSignal,
                                temperatureSignal));

        PhoenixUtil.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(0, armRoller));
    }

    public void updateInputs(GripperIOInputs inputs) {
        // Refresh all signals at once
        BaseStatusSignal.refreshAll(speedSignal, voltageSignal, currentSignal, temperatureSignal);

        inputs.speed = speedSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.current = currentSignal.getValueAsDouble();
        inputs.temperature = temperatureSignal.getValueAsDouble();
        inputs.sensor = gripperGPSensor.get();
    }

    public void setVoltage(double voltage) {
        armRoller.setVoltage(voltage);
    }
}
