package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class GripperIOSim implements GripperIO {
    private double leftVoltage;
    private double rightVoltage;
    private double speed;
    private LoggedNetworkBoolean gripperGPSensor =
            new LoggedNetworkBoolean("Gripper Sensor Sim", true);

    public void updateInputs(GripperIOInputs inputs) {
        inputs.voltageLeft = leftVoltage;
        inputs.voltageRight = rightVoltage;
        inputs.speedLeft = speed;
        inputs.hasPiece = gripperGPSensor.get();
    }

    public void setVoltage(double voltage) {
        this.leftVoltage = voltage;
        this.rightVoltage = voltage;
    }

    public void setVoltageLeft(double voltage) {
        this.leftVoltage = voltage;
    }

    public void setVoltageRight(double voltage) {
        this.rightVoltage = voltage;
    }
}
