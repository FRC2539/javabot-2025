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
    @AutoLogOutput public ScoringMode currentScoringMode = ScoringMode.Algae;
    @AutoLogOutput public Position lastPosition = Position.Home;

    public ModeManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;

        // goTo(targetPosition).schedule();

    }

    public static enum Position {
        L1(88, -1.65),
        L2(80, 0.09),

        L3(127, 0.09),
        L4(200, 0.16), // 0.14
        Algae2(83, 0.09), // 110

        Algae3(133, 0.09), // 190

        Handoff(92.15, -2.64), // -3// pre hat 94.463
        Home(90, -1.8),
        Start(1, -2.022),
        Climb(5, -2.022);

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

    public Command goTo(Position endPosition) {

        return Commands.runOnce(() -> targetPosition = endPosition, this)
                .andThen(
                        Commands.either(
                                Commands.either(
                                        Commands.sequence(
                                                arm.setPosition(endPosition.armHeight),
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
                                                arm.setPosition(endPosition.armHeight),
                                                Commands.waitUntil(() -> arm.isAtSetpoint())),
                                        () -> lastPosition == Position.Handoff),
                                Commands.sequence(
                                        elevator.setPosition(endPosition.elevatorHeight),
                                        // Commands.waitUntil(() ->
                                        // Math.abs(elevator.getPosition()
                                        // - targetPosition.elevatorHeight) <
                                        // 2).withTimeout(2),
                                        arm.setPosition(endPosition.armHeight),
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
