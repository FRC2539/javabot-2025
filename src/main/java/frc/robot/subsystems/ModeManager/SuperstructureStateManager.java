package frc.robot.subsystems.ModeManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState.Position;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class SuperstructureStateManager extends SubsystemBase {
    public class SuperstructureState {

        @FunctionalInterface
        private interface StateChecker {
            boolean isAtTarget(Position position, SuperstructureStateManager stateManager);
        }

        private static final StateChecker FALSE = (p, s) -> false;
        private static final StateChecker TRUE = (p, s) ->  true;
        private static final StateChecker DEFAULT = (p,s) -> {
            return (s.internalPosition == p);
        };



        public enum Position {
            Sussy(1,1,1,null),
            None(1, 1, 1, null, TRUE),
            Home(1, 1, 1, None),
            Preppy(1, 1, 1, None),
            PreppyNull(1,1,1,Preppy, TRUE),
            L1Prep(1, 1, 1, PreppyNull),
            L1(1, 1, 1, L1Prep),
            L2Prep(1, 1, 1, PreppyNull),
            L2(1, 1, 1, L2Prep),
            L3Prep(1, 1, 1, PreppyNull),
            L3(1, 1, 1, L3Prep),
            L4Prep(1, 1, 1, PreppyNull),
            L4(1, 1, 1, L4Prep),
            L1AlgaePrep(1, 1, 1, PreppyNull),
            L1Algae(1, 1, 1, L1AlgaePrep),
            L2AlgaePrep(1, 1, 1, PreppyNull),
            L2Algae(1, 1, 1, L2AlgaePrep),
            L3AlgaePrep(1, 1, 1, PreppyNull),
            L3Algae(1, 1, 1, L3AlgaePrep),
            L4AlgaePrep(1, 1, 1, PreppyNull),
            L4Algae(1, 1, 1, L4AlgaePrep),
            SourcePrep(1, 1, 1, None),
            Source(1, 1, 1, SourcePrep);

            public double elevatorheight;
            public double armheight;
            public double wristrotation;
            public Position position;
            public Position parent;
            public StateChecker isAtTarget;

            private Position(
                    double elevatorheight,
                    double armheight,
                    double wristRotation,
                    Position parent,
                    StateChecker isAtTarget) {
                this.armheight = armheight;
                this.elevatorheight = elevatorheight;
                this.position = this;
                this.parent = parent;
                this.wristrotation = wristRotation;
                this.isAtTarget = isAtTarget;
            }

            private Position(
                    double elevatorheight,
                    double armheight,
                    double wristRotation,
                    Position parent) {
                        this(elevatorheight, armheight, wristRotation, parent, DEFAULT);
                    }

            public boolean isAtTarget(SuperstructureStateManager stateManager) {
                return isAtTarget.isAtTarget(this, stateManager);
            }
        }
    }

    private SuperstructureState.Position targetPostition = SuperstructureState.Position.None;
    private SuperstructureState.Position lastPosition = SuperstructureState.Position.None;

    private List<SuperstructureState.Position> outList = new ArrayList<>();
    // public List<SuperstructureState.Position> inList = new ArrayList<>();
    private ElevatorSubsystem ElevatorSubsystem;
    private ArmSubsystem ArmSubsystem;

    public void periodic() {
        Logger.recordOutput("Superstructure/Target", targetPostition);
        Logger.recordOutput("Superstructure/Last", lastPosition);
        Logger.recordOutput("Superstructure/Internal", internalPosition);
        Logger.recordOutput("Superstructure/InternalPersistant", internalPosition == Position.Sussy ? null : internalPosition);
        Logger.recordOutput("Superstructure/OutList", outList.toArray(new SuperstructureState.Position[0]));
    }

    private void setFinalTarget(SuperstructureState.Position myPosition) {
        outList.clear();
        // Set the [ (A') => (C') ] target of the system (initialize pathing command)
        boolean found = false;

        // outList.add(0,myPosition);

        for (int i = 0; i < 20; i++) {
            // SuperstructureState head = SuperstructureState.posDictionary.get(myPosition);
            outList.add(0, myPosition);
            if (myPosition == SuperstructureState.Position.None) {
                found = true;
                break;
            }
            myPosition = myPosition.parent;
        }

        if (!found || myPosition == null) {
            throw new RuntimeException("AY CARAMBA. (Your state machine didnt resolve.)");
        }
    }

    private void updateTarget(SuperstructureState.Position myPosition) {
        if (myPosition == lastPosition) {
            lastPosition = targetPostition;
        }
        targetPostition = myPosition;
    }

    // private void setCurrentTarget(SuperstructureState.Position myPosition) {

    //     // Set the [ (A) => (A') ] target of the system (initialize movement command)
    // }

    // public Command inList(){
    //    for()

    // }

    /** Nodes must either be identical or neighbors. */
    private SuperstructureState.Position getChilderNodeInBranch(
            SuperstructureState.Position nodeA, SuperstructureState.Position nodeB) {
        var higherPose = nodeA;
        if (nodeB.parent == nodeA) {
            higherPose = nodeB;
        }
        return higherPose;
    }

    private SuperstructureState.Position internalPosition = Position.Sussy;
    private Command internalGoToPosition(SuperstructureState.Position myPosition) {
        return Commands.idle().beforeStarting(() -> internalPosition = Position.Sussy).withTimeout(1).andThen(() -> internalPosition = myPosition).andThen(Commands.idle());
    }

    public Command moveToPosition(SuperstructureState.Position myPosition) {
        Command setFinalTarget = Commands.runOnce(() -> setFinalTarget(myPosition));
        Command followInPath =
                followInPath()
                        .until(
                                () -> {
                                    return outList.contains(lastPosition)
                                            && outList.contains(targetPostition);
                                });

        Command clearOutPath =
                Commands.runOnce(
                        () -> {
                            var childerPose = getChilderNodeInBranch(lastPosition, targetPostition);

                            for (int i = 0; i < 20; i++) {
                                if (outList.contains(childerPose)
                                        && outList.get(0) != childerPose) {
                                    outList.remove(0);
                                }
                            }
                        });
        Command followOutPath = followOutPath();
        Command outputCommand = setFinalTarget.andThen(followInPath).andThen(clearOutPath).andThen(followOutPath);
        outputCommand.addRequirements(this);
        return outputCommand;
    }

    private Command followOutPath() {
        return (Commands.defer(
                        () -> {
                            SuperstructureState.Position nextPose = outList.remove(0);
                            if (outList.size() == 0) {
                                return internalGoToPosition(nextPose)
                                        .beforeStarting(() -> updateTarget(nextPose));
                            }
                            return internalGoToPosition(nextPose)
                                    .beforeStarting(() -> updateTarget(nextPose))
                                    .until(() -> nextPose.isAtTarget(this))
                                    .andThen(
                                            () -> {
                                                lastPosition = nextPose;
                                            });
                        },
                        new HashSet<>()))
                .repeatedly();
    }

    private Command followInPath() {
        return (Commands.defer(
                        () -> {
                            SuperstructureState.Position nextPose =
                                    getChilderNodeInBranch(lastPosition, targetPostition).parent;
                            if (nextPose == null) {
                                return internalGoToPosition(Position.None);
                            }
                            return internalGoToPosition(nextPose)
                                    .beforeStarting(() -> updateTarget(nextPose))
                                    .until(() -> nextPose.isAtTarget(this))
                                    .andThen(() -> lastPosition = nextPose);
                        },
                        new HashSet<>()))
                .repeatedly();
    }

    public SuperstructureStateManager(ElevatorSubsystem elevatorheight, ArmSubsystem armSubsystem) {
        ElevatorSubsystem = elevatorheight;
        ArmSubsystem = armSubsystem;
    }
}
