package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmPivotIO {
    public void updateInputs(ArmPivotIOInputs inputs);

    @AutoLog
    public class ArmPivotIOInputs {
        public double position = 0;
        public double speed = 0;
        public double voltage = 0;
    }

    public void setPosition(double position);

    public void setVoltage(double voltage);
}
