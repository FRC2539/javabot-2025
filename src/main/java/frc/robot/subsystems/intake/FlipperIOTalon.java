package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.PhoenixUtil;

public class FlipperIOTalon implements FlipperIO {
    private final TalonFX flipperMotor =
            new TalonFX(IntakeConstants.idFlipper, IntakeConstants.canbusFlipper);
    private final PositionVoltage openPV = new PositionVoltage(IntakeConstants.openPosit);
    private final PositionVoltage closedPV = new PositionVoltage(IntakeConstants.closedPosit);

    // Status signals for efficient updates
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Current> currentSignal;

    public FlipperIOTalon() {
        flipperMotor.setPosition(0);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(IntakeConstants.openPosit)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(IntakeConstants.closedPosit);

        flipperMotor
                .getConfigurator()
                .apply(
                        new TalonFXConfiguration()
                                .withCurrentLimits(IntakeConstants.currentLimit)
                                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs));

        flipperMotor.setNeutralMode(NeutralModeValue.Brake);

        // Initialize status signals
        positionSignal = flipperMotor.getPosition();
        voltageSignal = flipperMotor.getMotorVoltage();
        temperatureSignal = flipperMotor.getDeviceTemp();
        currentSignal = flipperMotor.getStatorCurrent();

        // Set update frequency and optimize bus utilization
        PhoenixUtil.tryUntilOk(
                5,
                () ->
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                positionSignal,
                                voltageSignal,
                                temperatureSignal,
                                currentSignal));

        PhoenixUtil.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(0, flipperMotor));
    }

    public void updateInputs(FlipperIOInputs inputs) {
        // Refresh all signals at once
        BaseStatusSignal.refreshAll(
                positionSignal, voltageSignal, temperatureSignal, currentSignal);

        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.temperature = temperatureSignal.getValueAsDouble();
        inputs.current = currentSignal.getValueAsDouble();
    }

    public void setOpen() {

        flipperMotor.setControl(openPV);
    }

    public void setClose() {
        flipperMotor.setControl(closedPV);
    }

    public void setVoltage(double voltage) {
        flipperMotor.setVoltage(voltage);
    }

    public void resetPosition(double position) {
        flipperMotor.setPosition(position);
    }
}
