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
    @AutoLogOutput public Position targetPosition = Position.Home;
    @AutoLogOutput public ScoringMode currentScoringMode = ScoringMode.zero;
    @AutoLogOutput public Position lastPosition = Position.Home;

    public ModeManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;

        // goTo(targetPosition).schedule();

    }

    public static enum Position {
        L1(82, 0.55), // -1.65 arm value
        L2(76, 2.323), // 0.09 arm value //2.213
        ToryL2(113, 2.323),
        L3(125, 2.323), // 0.09 arm value
        L4(200, 2.45), // 0.14 // 0.16 arm value //2.213
        Algae2(83, 2.323), // 110 //0.09 arm value

        Algae3(133, 2.323), // 190 // 0.09 arm value

        Handoff(89, -0.55), // -0.472 // -3// pre hat 94.463 //-2.64 arm value
        Home(90, 0.222), // 0.07 // -1.8 arm value
        Start(1, 0.222), // -2.022 arm value
        Climb(5, 0.222); // -2.022 arm value

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
        zero,
        Algae,
        LeftCoral,
        RightCoral
    }

    public Command moveArm(Position position) {
        return Commands.either(
                arm.setPositionL1(position.armHeight),
                arm.setPosition(position.armHeight),
                () -> {
                    return position == Position.L1 || position == Position.Home;
                });
    }

    public Command goTo(Position endPosition) {

        return Commands.runOnce(() -> targetPosition = endPosition, this)
                .andThen(
                        Commands.either(
                                Commands.either(
                                        Commands.sequence(
                                                moveArm(endPosition),
                                                Commands.waitUntil(() -> arm.isAtSetpoint()),
                                                elevator.setPosition(endPosition.elevatorHeight),
                                                Commands.waitUntil(
                                                        () ->
                                                                (Math.abs(
                                                                                elevator
                                                                                                .getPosition()
                                                                                        - endPosition
                                                                                                .elevatorHeight)
                                                                        < 2))),
                                        Commands.sequence(
                                                elevator.setPosition(endPosition.elevatorHeight),
                                                Commands.waitUntil(
                                                        () ->
                                                                (Math.abs(
                                                                                elevator
                                                                                                .getPosition()
                                                                                        - endPosition
                                                                                                .elevatorHeight)
                                                                        < 2)),
                                                moveArm(endPosition),
                                                Commands.waitUntil(() -> arm.isAtSetpoint())),
                                        () -> lastPosition == Position.Handoff),
                                Commands.sequence(
                                        elevator.setPosition(endPosition.elevatorHeight),
                                        // Commands.waitUntil(() ->
                                        // Math.abs(elevator.getPosition()
                                        // - targetPosition.elevatorHeight) <
                                        // 2).withTimeout(2),
                                        moveArm(endPosition),
                                        Commands.waitUntil(() -> arm.isAtSetpoint()),
                                        Commands.waitUntil(
                                                () ->
                                                        (Math.abs(
                                                                        elevator.getPosition()
                                                                                - endPosition
                                                                                        .elevatorHeight)
                                                                < 2))),
                                () ->
                                        lastPosition == Position.Handoff
                                                || endPosition == Position.Handoff))
                .andThen(Commands.runOnce(() -> lastPosition = endPosition, this))
                .andThen(Commands.idle(this));
    }

    // return Commands.either(
    //     Commands.sequence(
    //         elevator.setPosition(endPosition.elevatorHeight),
    //         //Commands.waitUntil(() -> Math.abs(elevator.getPosition() -
    // targetPosition.elevatorHeight) < 2).withTimeout(2),
    //         arm.setPosition(endPosition.armHeight).until(() -> arm.isAtSetpoint())
    //     ),
    //     Commands.sequence(
    //         arm.setPosition(endPosition.armHeight).until(() -> arm.isAtSetpoint()),
    //         elevator.setPosition(endPosition.elevatorHeight)),
    //         //Commands.waitUntil(() -> (Math.abs(elevator.getPosition() -
    // endPosition.elevatorHeight) < 2))),

    //     () -> endPosition != Position.Handoff || endPosition == Position.Handoff
    // );

    // }

    public void setScoringMode(ScoringMode mode) {
        this.currentScoringMode = mode;
    }

    public ScoringMode getCurrentScoringMode() {
        return this.currentScoringMode;
    }

    @AutoLogOutput
    public boolean isArmAtPosition() {
        return arm.isAtSetpoint();
    }

    @AutoLogOutput
    public boolean isElevatorAtPosition() {
        return (Math.abs(elevator.getPosition() - targetPosition.elevatorHeight) < 2);
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

    @Override
    public void periodic() {}
}
