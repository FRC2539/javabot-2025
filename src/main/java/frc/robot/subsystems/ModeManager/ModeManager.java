package frc.robot.subsystems.ModeManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AligningConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ModeManager extends SubsystemBase {

    public ElevatorSubsystem elevator;
    public ArmSubsystem arm;

    public ModeManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public static class State {
        private static LoggedNetworkNumber elevatorHeightLogged =
                new LoggedNetworkNumber("Elevator Height", 170);
        private static LoggedNetworkNumber armHeightLogged =
                new LoggedNetworkNumber("Arm Height", 0);

        public static enum Position {
            Home(170.0, 1.58),
            L1(130.0, 0),
            L2(90.0, -1.58),
            L3(160.0, -1.58),
            L4(297.0, -1.58),
            Quick2(150.0, 0.0),
            Quick3(250.0, 0.0),
            Handoff(160.0, 1.58),
            HandoffFlipped(160.0, 1.58),
            Source(130.0, 0.0),
            Start(0.0, -1.58),
            Climb(10.0, -1.58),
            Tunable(170.0, 1.58);

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
    }

    public enum CoralAlgaeMode {
        LeftCoral,
        RightCoral,
        Algae;
    }

    private CoralAlgaeMode currentMode = CoralAlgaeMode.LeftCoral;

    public void setMode(CoralAlgaeMode mode) {
        currentMode = mode;
    }

    public double getAligningOffset() {
        if (currentMode == CoralAlgaeMode.LeftCoral) {
            return AligningConstants.leftOffset;
        } else if (currentMode == CoralAlgaeMode.RightCoral) {
            return AligningConstants.rightOffset;
        } else {
            return AligningConstants.centerOffset;
        }
    }

    public Command setGoal(frc.robot.subsystems.ModeManager.ModeManager.State.Position position) {

        return Commands.parallel(
                Commands.runOnce(() -> elevator.setPosition(position.elevatorHeight()), elevator),
                Commands.runOnce(() -> arm.setPosition(position.armHeight()), arm));
    }
}
