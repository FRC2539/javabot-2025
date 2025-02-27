package frc.robot.subsystems.ModeManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.AligningConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import static edu.wpi.first.units.Units.PoundInch;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

public class ModeManager extends SubsystemBase {
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    @AutoLogOutput public Position targetPosition = Position.Home;
    @AutoLogOutput public ScoringMode currentScoringMode = ScoringMode.Algae;

    public ModeManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;

        //goTo(targetPosition).schedule();

    }

    public static enum Position {
        L1(70, 1.1),
        L2(95, 2.2),
        L3(180, 2.2),
        L4(297, 0.09),
        Algae2(130, 1.1),

        Algae3(130, 1.1),
        Handoff(154, -2.58),
        Home(170, -1.8),
        Start(0, -2.022),
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

    public Command goTo(Position endPosition) {
        
            return Commands.either(
                Commands.sequence(
                    elevator.setPosition(endPosition.elevatorHeight),
                    //Commands.waitUntil(() -> Math.abs(elevator.getPosition() - targetPosition.elevatorHeight) < 2).withTimeout(2),
                    arm.setPosition(endPosition.armHeight)
                ),
                Commands.sequence(
                    arm.setPosition(endPosition.armHeight),
                    Commands.waitUntil(() -> arm.isAtSetpoint()),
                    elevator.setPosition(endPosition.elevatorHeight),
                    Commands.waitUntil(() -> (Math.abs(elevator.getPosition() - endPosition.elevatorHeight) < 2))),
                    
                
                () -> endPosition != Position.Handoff || endPosition == Position.Handoff
            );

    }

    public void setScoringMode(ScoringMode mode) {
        this.currentScoringMode = mode;
    }

    public ScoringMode getCurrentScoringMode() {
        return this.currentScoringMode;
    }

    @AutoLogOutput
    public boolean isAtPosition() {
        System.out.println("at set:" + arm.isAtSetpoint() + "  " + arm.getPosition());
        System.out.println("elev set:" +(Math.abs(elevator.getPosition() - targetPosition.elevatorHeight)
        < 0.5));
        return arm.isAtSetpoint()
                && (Math.abs(elevator.getPosition() - targetPosition.elevatorHeight)
                        < 0.5); // TODO: TUNE
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
}
