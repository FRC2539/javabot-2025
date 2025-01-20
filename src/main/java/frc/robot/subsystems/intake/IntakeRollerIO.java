package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
    public void updateInputs(IntakeRollerIOInputs inputs);

    @AutoLog
    public class IntakeRollerIOInputs {
        public double speed = 0;
        public double voltage = 0;
        public double temperature = 0;
        public double current = 0;
    }

    public void setVoltage(double voltage);
}
