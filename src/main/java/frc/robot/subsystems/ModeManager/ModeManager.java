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
    @AutoLogOutput private Position targetPosition;
    @AutoLogOutput private ScoringMode currentScoringMode = ScoringMode.Algae;

    public ModeManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;

        goTo(Position.Start);
    }

    public static enum Position {
        L1(130, 1.1),
        L2(90, 2.2),
        L3(160, 2.2),
        L4(248.6, 0.09),
        Algae2(130, 1.1),




        Algae3(130, 1.1),
        Handoff(155.6, -2.6),
        Home(144, -1.86),
        Start(0, -1.86),
        Climb(160, -1.86);

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

    public Command goTo(Position position) {
        this.targetPosition = position;
        // return Commands.sequence(
        //         Commands.runOnce(() -> {this.targetPosition = position;}, this),
        //         arm.setPosition(targetPosition.armHeight),
        //         Commands.waitUntil(() -> arm.isAtSetpoint()),
        //         elevator.setPosition(targetPosition.elevatorHeight));
        return Commands.sequence(Commands.runOnce(() -> {this.targetPosition = position;}, this), 
        elevator.setPosition(targetPosition.elevatorHeight))
        .andThen(Commands.waitSeconds(5))
        .andThen(arm.setPosition(targetPosition.armHeight));
    }

    public void setScoringMode(ScoringMode mode) {
        this.currentScoringMode = mode;
    }

    public ScoringMode getCurrentScoringMode() {
        return this.currentScoringMode;
    }

    @AutoLogOutput
    public boolean isAtPosition() {
        System.out.println(
                arm.isAtSetpoint()
                        + "  "
                        + arm.getPosition());
        return arm.isAtSetpoint()
                && (Math.abs(elevator.getPosition() - targetPosition.elevatorHeight)
                        < 0.5); // TODO: TUNE
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
