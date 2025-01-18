package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FlipperIOSim implements FlipperIO {

    private LoggedNetworkNumber simMotor = new LoggedNetworkNumber("simMotor");
    private LoggedNetworkNumber simMotorPosition = new LoggedNetworkNumber("simMotorPosition");

    public void updateInputs(FlipperIOInputs inputs) {
        inputs.position = simMotorPosition.get();
        inputs.voltage = simMotor.get();
    }

    public void setOpen() {
        simMotorPosition.set(100);
    }

    public void setClose() {
        simMotorPosition.set(0);
    }

    public void setVoltage(double voltage) {
        simMotor.set(voltage);
    }
}
