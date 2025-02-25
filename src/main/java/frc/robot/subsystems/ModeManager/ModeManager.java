package frc.robot.subsystems.ModeManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AligningConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class ModeManager extends SubsystemBase {
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    @AutoLogOutput private ScoringMode currentScoringMode = ScoringMode.Algae;

    public ModeManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public static enum Position {
        L1(130, 1.1),
        L2(90, 2.2),
        L3(160, 2.2),
        L4(297, 1.7),
        Algae2(130, 1.1),
        Algae3(130, 1.1),
        Handoff(130, 1.1),
        Home(130, 1.1),
        Start(130, 1.1),
        Climb(130, 1.1);

        private double elevatorHeight;
        private double armHeight;

        private Position(double elevatorHeight, double armHeight) {
            this.elevatorHeight = elevatorHeight;
            this.armHeight = armHeight;
        }

        public double elevatorHeight() {
            return elevatorHeight;
        }

        public double armHeight() {
            return armHeight;
        }
    }

    public static enum ScoringMode {
        Algae,
        LeftCoral,
        RightCoral
    }

    public Command goTo(Position targetPosition) {
        return Commands.parallel(
                arm.setPosition(targetPosition.armHeight),
                elevator.setPosition(targetPosition.elevatorHeight));
    }

    public void setScoringMode(ScoringMode mode) {
        this.currentScoringMode = mode;
    }

    public ScoringMode getCurrentScoringMode() {
        return this.currentScoringMode;
    }

    public double getAligningOffset() {

        switch (currentScoringMode) {
            case Algae:
                return AligningConstants.centerOffset;
            case LeftCoral:
                return AligningConstants.leftOffset;
            case RightCoral:
                return AligningConstants.rightOffset;
        }

        return 0;
    }
}
