package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.WristConstants;

public class WristIOSim implements WristIO {
    private double position = 0;
    private double voltage = 0;

    private double reference = 0;
    private boolean usingPosition = false;

    private PIDController controller =
            new PIDController(
                    WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);

    public void updateInputs(WristIOInputs inputs) {
        if (usingPosition) {
            voltage = controller.calculate(position, reference);
        }

        position += 0.02 * voltage;

        inputs.position = position;
        inputs.throughboreEncoderPosition = position;
        inputs.throughboreConnected = true;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        usingPosition = false;
    }

    public void setPositionControl(double reference) {
        this.reference = reference;
        usingPosition = true;
    }
}
