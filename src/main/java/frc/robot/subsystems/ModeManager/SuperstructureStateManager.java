package frc.robot.subsystems.ModeManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState.Position;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureStateManager extends SubsystemBase {

    public final LoggedMechanismLigament2d m_elevator;
    public final LoggedMechanismLigament2d m_wrist;

    private static final double kElevatorMinimumLength = 0.5;

    public class SuperstructureState {

        @FunctionalInterface
        public interface StateChecker {
            boolean isAtTarget(Position position, SuperstructureStateManager stateManager);
        }

        public static final StateChecker FALSE = (p, s) -> false;
        public static final StateChecker TRUE = (p, s) -> true;
        public static final StateChecker DEFAULT =
                (p, s) -> {
                    // return (s.internalPosition == p);
                    double armPosition = s.armSubsystem.getPosition();
                    double wristPosition = s.wristSubsystem.getFlippedPosition();
                    double elevatorPosition = s.elevatorSubsystem.getPosition();

                    if ((Math.abs(armPosition - p.armheight) < 0.1)
                            && (Math.abs(wristPosition - p.wristrotation) < 0.1)
                            && (Math.abs(elevatorPosition - p.elevatorheight) < 0.1)) {
                        return true;

                    } else return false;
                };
        public static final StateChecker AUTO = DEFAULT;

        public enum Position {
            Sussy(1, 1, 1, null),
            None(1, 1, 1, null, TRUE, false),
            Home(1, 0, 1, None),
            Handoff(1, 0, 1, None),
            Icecream(1, 1, 1, None),
            Quick34(4, 2, 1, None),
            Quick23(3, 3, 1, None),
            Preppy(1, 2, 1, None),
            PreppyNull(1, 3, 1, Preppy, TRUE, false),
            L1Prep(2, 2, 1, PreppyNull),
            L1(2, 1, 4, L1Prep),
            L2Prep(3, 2, 1, PreppyNull),
            L2(3, 1, 1, L2Prep),
            L3Prep(4, 2, 1, PreppyNull),
            L3(4, 1, 1, L3Prep),
            L4Prep(5, 2, 1, PreppyNull),
            L4(5, 1, 1, L4Prep),
            L1AlgaePrep(2, 2, 1, PreppyNull),
            L1Algae(2, 1, 1, L1AlgaePrep),
            L2AlgaePrep(3, 2, 1, PreppyNull),
            L2Algae(3, 1, 1, L2AlgaePrep),
            L3AlgaePrep(4, 2, 1, PreppyNull),
            L3Algae(4, 1, 1, L3AlgaePrep),
            L4AlgaePrep(5, 2, 1, PreppyNull),
            L4Algae(5, 1, 1, L4AlgaePrep),
            SourcePrep(1, -2, 1, None),
            Source(0, -2, 1, SourcePrep);

            public double elevatorheight;
            public double armheight;
            public double wristrotation;
            public Position position;
            public Position parent;
            public StateChecker isAtTarget;
            public boolean realPosition;

            private Position(
                    double elevatorheight,
                    double armheight,
                    double wristRotation,
                    Position parent,
                    StateChecker isAtTarget,
                    boolean realPosition) {
                this.armheight = armheight;
                this.elevatorheight = elevatorheight;
                this.position = this;
                this.parent = parent;
                this.wristrotation = wristRotation;
                this.isAtTarget = isAtTarget;
                this.realPosition = realPosition;
            }

            private Position(
                    double elevatorheight,
                    double armheight,
                    double wristRotation,
                    Position parent,
                    StateChecker isAtTarget) {
                this(elevatorheight, armheight, wristRotation, parent, DEFAULT, true);
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

    private enum CoralAlgaeMode {
        LeftCoral,
        RightCoral,
        Algae,
        ArmWrist;
    }

    private CoralAlgaeMode coralAlgaeMode = CoralAlgaeMode.LeftCoral;

    private Pose2d lastScoringPose = Pose2d.kZero;

    public void setLastScoringPose(Pose2d pose) {
        lastScoringPose = pose;
    }

    public Pose2d getLastScoringPose() {
        return lastScoringPose;
    }

    public final Trigger LEFT_CORAL = new Trigger(() -> coralAlgaeMode == CoralAlgaeMode.LeftCoral);
    public final Trigger RIGHT_CORAL =
            new Trigger(() -> coralAlgaeMode == CoralAlgaeMode.RightCoral);
    public final Trigger ALGAE = new Trigger(() -> coralAlgaeMode == CoralAlgaeMode.Algae);
    public final Trigger ARMWRIST = new Trigger(() -> coralAlgaeMode == CoralAlgaeMode.ArmWrist);

    public Command setLeftCoralMode() {
        return Commands.runOnce(() -> coralAlgaeMode = CoralAlgaeMode.LeftCoral);
    }

    public Command setRightCoralMode() {
        return Commands.runOnce(() -> coralAlgaeMode = CoralAlgaeMode.RightCoral);
    }

    public Command setAlgaeMode() {
        return Commands.runOnce(() -> coralAlgaeMode = CoralAlgaeMode.Algae);
    }

    public Command setArmWristMode() {
        return Commands.runOnce(() -> coralAlgaeMode = CoralAlgaeMode.ArmWrist);
    }

    // public List<SuperstructureState.Position> inList = new ArrayList<>();
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;

    public void periodic() {
        Logger.recordOutput("Superstructure/Target", targetPostition);
        Logger.recordOutput("Superstructure/Last", lastPosition);
        Logger.recordOutput("Superstructure/Internal", internalPosition);
        Logger.recordOutput(
                "Superstructure/InternalPersistant",
                internalPosition == Position.Sussy ? null : internalPosition);
        Logger.recordOutput(
                "Superstructure/OutList", outList.toArray(new SuperstructureState.Position[0]));

        Logger.recordOutput("Superstructure/Algae", ALGAE.getAsBoolean());
        Logger.recordOutput("Superstructure/LeftCoral", LEFT_CORAL.getAsBoolean());
        Logger.recordOutput("Superstructure/RightCoral", RIGHT_CORAL.getAsBoolean());

        m_elevator.setLength(elevatorSubsystem.getPosition());
        m_wrist.setAngle(Math.toDegrees(armSubsystem.getPosition()) + 180);
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

    // // Set the [ (A) => (A') ] target of the system (initialize movement command)
    // }

    // public Command inList(){
    // for()

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

    /*
     * The `internalGoToPosition` command needs to actually command the elevator and
     * the wrist and the arm to go to a position
     */
    private Command internalGoToPosition(SuperstructureState.Position myPosition) {
        if (myPosition.realPosition) {
            return elevatorSubsystem
                    .setPosition(myPosition.elevatorheight)
                    .alongWith(armSubsystem.setPosition(myPosition.armheight))
                    .alongWith(wristSubsystem.setPosition(myPosition.wristrotation));
        } else {
            return Commands.idle();
        }
        // return Commands.idle().beforeStarting(() -> internalPosition =
        // Position.Sussy).withTimeout(1).andThen(() -> internalPosition =
        // myPosition).andThen(Commands.idle());
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
        Command outputCommand =
                setFinalTarget.andThen(followInPath).andThen(clearOutPath).andThen(followOutPath);
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
                        Set.of(elevatorSubsystem, armSubsystem, wristSubsystem)))
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
                        Set.of(elevatorSubsystem, armSubsystem, wristSubsystem)))
                .repeatedly();
    }

    public SuperstructureStateManager(
            ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem,
            WristSubsystem wristSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;

        LoggedMechanism2d mech = new LoggedMechanism2d(3, 6);

        LoggedMechanismRoot2d root = mech.getRoot("Elevator", 2, 0);

        m_elevator = new LoggedMechanismLigament2d("elevator", kElevatorMinimumLength, 90);

        m_wrist =
                new LoggedMechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kAliceBlue));

        m_elevator.append(m_wrist);
        root.append(m_elevator);

        SmartDashboard.putData("Mech2d", mech);

        Command defaultcom =
                Commands.either(
                        Commands.idle(),
                        moveToPosition(Position.Home).asProxy(),
                        () -> !RobotState.isAutonomous());
        defaultcom.addRequirements(this);
        setDefaultCommand(defaultcom);
    }
}
