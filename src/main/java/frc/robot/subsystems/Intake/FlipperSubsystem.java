package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.FlipperIO.FlipperIOInputs;
import org.littletonrobotics.junction.Logger;

public class FlipperSubsystem extends SubsystemBase {

    private FlipperIO piviotIO;
    private FlipperIOInputs flipperInputs = new FlipperIOInputs();

    private final double lowerLimit = 0;
    private final double upperLimit = 100;

    public FlipperSubsystem(FlipperIO flipperIO) {
        this.piviotIO = flipperIO;
    }

    public void periodic() {

        piviotIO.updateInputs(flipperInputs);

        Logger.recordOutput("Fipper/Position", flipperInputs.position);

        if (flipperInputs.voltage < 0 && flipperInputs.position <= lowerLimit) {
            this.piviotIO.setVoltage(0);
        }

        if (flipperInputs.voltage > 0 && flipperInputs.position >= upperLimit) {
            this.piviotIO.setVoltage(0);
        }
    }

    public Command openIntake() {
        return run(() -> piviotIO.setOpen());
    }

    public Command closeIntake() {
        return run(() -> piviotIO.setClose());
    }
}
