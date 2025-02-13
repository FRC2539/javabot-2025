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
import frc.robot.subsystems.chute.ChuteSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.Elastic;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class SuperstructureStateManager extends SubsystemBase {

    public final LoggedMechanismLigament2d m_elevator;
    public final LoggedMechanismLigament2d m_wrist;

    private static final double kElevatorMinimumLength = 0.5;

    public static class SuperstructureState {
        private static LoggedNetworkNumber elevatorHeightLogged =
                new LoggedNetworkNumber("Elevator Height", 0);
        private static LoggedNetworkNumber armHeightLogged =
                new LoggedNetworkNumber("Arm Height", 0);
        private static LoggedNetworkNumber wristRotationLogged =
                new LoggedNetworkNumber("Wrist Rotation", 0);
        private static LoggedNetworkString parentLogged =
                new LoggedNetworkString("Superstructure Parent", "Home");
        private static LoggedNetworkString buttonTargetLogged =
                new LoggedNetworkString("Superstructure Button Target", "Tunable");

        @FunctionalInterface
        public interface StateChecker {
            boolean checksOut(Position position, SuperstructureStateManager stateManager);
        }

        public static final StateChecker FALSE = (p, s) -> false;
        public static final StateChecker TRUE = (p, s) -> true;
        public static final StateChecker DEFAULT =
                (p, s) -> {
                    // return (s.internalPosition == p);
                    double armPosition = s.armSubsystem.getPosition();
                    double wristPosition = s.wristSubsystem.getFlippedPosition();
                    double elevatorPosition = s.elevatorSubsystem.getPosition();

                    if ((Math.abs(armPosition - p.armHeight()) < 0.1)
                            && (Math.abs(wristPosition - p.wristRotation()) < 0.1)
                            && (Math.abs(elevatorPosition - p.elevatorHeight()) < 0.1)) {
                        return true;

                    } else return false;
                };
        public static final StateChecker AUTO = DEFAULT;

        public static enum Position {
            Sussy(1, 1, 1, null),
            None(1, 1, 1, null, TRUE, FALSE),
            Home(1, 0, 1, None),
            ChuteDown(0, 0, 0, None),
            ChuteUp(0, 0, 0, None),
            ChuteDownNull(0, 0, 0, ChuteDown),
            ChuteUpNull(0, 0, 0, ChuteUp),
            HandoffPrep(1, 0, 1, ChuteDownNull),
            Handoff(1, 0, 1, HandoffPrep),
            Quick34(4, 2, 1, None),
            Quick23(3, 3, 1, None),
            PointUp(1, 2, 1, None),
            UpZoneNull(1, 3, 1, PointUp, TRUE, FALSE),
            L1Prep(2, 2, 1, ChuteDownNull),
            L1(2, 1, 4, L1Prep),
            L2Prep(3, 2, 1, UpZoneNull),
            L2(3, 1, 1, L2Prep),
            L3Prep(4, 2, 1, UpZoneNull),
            L3(4, 1, 1, L3Prep),
            L4Prep(5, 2, 1, UpZoneNull),
            L4(5, 1, 1, L4Prep),
            L1AlgaePrep(2, 2, 1, UpZoneNull),
            L1Algae(2, 1, 1, L1AlgaePrep),
            L2AlgaePrep(3, 2, 1, UpZoneNull),
            L2Algae(3, 1, 1, L2AlgaePrep),
            L3AlgaePrep(4, 2, 1, UpZoneNull),
            L3Algae(4, 1, 1, L3AlgaePrep),
            L4AlgaePrep(5, 2, 1, UpZoneNull),
            L4Algae(5, 1, 1, L4AlgaePrep),
            Source(1, -2, 1, None),
            AlgaeHome(1, 1, 1, None),
            Climb(1, 1, 1, ChuteDownNull),
            Processor(1, 1, 1, ChuteDownNull),
            IcecreamCoral(1, 1, 1, ChuteDownNull),
            IcecreamAlgae(1, 1, 1, ChuteDownNull),
            Tunable(0, 0, 0, None, FALSE);

            private double elevatorHeight;
            private double armHeight;
            private double wristRotation;
            public Position position;
            private Position parent;
            public StateChecker isAtTarget;
            public StateChecker realPosition;

            private Position(
                    double elevatorHeight,
                    double armHeight,
                    double wristRotation,
                    Position parent,
                    StateChecker isAtTarget,
                    StateChecker realPosition) {
                this.armHeight = armHeight;
                this.elevatorHeight = elevatorHeight;
                this.position = this;
                this.parent = parent;
                this.wristRotation = wristRotation;
                this.isAtTarget = isAtTarget;
                this.realPosition = realPosition;
            }

            private Position(
                    double elevatorHeight,
                    double armHeight,
                    double wristRotation,
                    Position parent,
                    StateChecker isAtTarget) {
                this(elevatorHeight, armHeight, wristRotation, parent, DEFAULT, TRUE);
            }

            private Position(
                    double elevatorHeight,
                    double armHeight,
                    double wristRotation,
                    Position parent) {
                this(elevatorHeight, armHeight, wristRotation, parent, DEFAULT);
            }

            public boolean isAtTarget(SuperstructureStateManager stateManager) {
                return isAtTarget.checksOut(this, stateManager);
            }

            public boolean isRealPosition(SuperstructureStateManager stateManager) {
                return realPosition.checksOut(this, stateManager);
            }

            // #region Pointer Methods
            public double elevatorHeight() {
                if (this == Position.Tunable) {
                    elevatorHeight =
                            SuperstructureState.elevatorHeightLogged
                                    .get(); // ref: tunable variable armHeight
                }
                return elevatorHeight;
            }

            public double armHeight() {
                if (this == Position.Tunable) {
                    armHeight =
                            SuperstructureState.armHeightLogged
                                    .get(); // ref: tunable variable armHeight
                }
                return armHeight;
            }

            public double wristRotation() {
                if (this == Position.Tunable) {
                    wristRotation = SuperstructureState.wristRotationLogged.get();
                    // ref: tunable variable wristRotation
                }
                return wristRotation;
            }

            public Position parent() {
                if (this == Position.Tunable) {
                    parent = Position.valueOf(parentLogged.get()); // ref: tunable variable parent
                }
                return parent;
            }
            // #endregion

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

    private boolean chuteCanMove = false;

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
    private ChuteSubsystem chuteSubsystem;

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
        Logger.recordOutput(
                "Superstructure/AnyCoral", LEFT_CORAL.getAsBoolean() || RIGHT_CORAL.getAsBoolean());

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
            myPosition = myPosition.parent();
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
        if (nodeB.parent() == nodeA) {
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
        if (myPosition == Position.ChuteDownNull) {
            chuteCanMove = false;
        }
        if (myPosition == Position.ChuteUpNull) {
            chuteCanMove = false;
        }
        if (myPosition == Position.None) {
            chuteCanMove = true;
        }

        if (myPosition.isRealPosition(this)) {
            return elevatorSubsystem
                    .setPosition(myPosition.elevatorHeight())
                    .alongWith(armSubsystem.setPosition(myPosition.armHeight()))
                    .alongWith(wristSubsystem.setPosition(myPosition.wristRotation()));
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

        Command chuteUp =
                Commands.waitUntil(() -> chuteCanMove)
                        .andThen(chuteSubsystem.moveChuteUp()::schedule);
        Command chuteDown =
                Commands.waitUntil(() -> chuteCanMove)
                        .andThen(chuteSubsystem.moveChuteDown()::schedule);

        Command chuteMover =
                chuteUp.onlyIf(() -> outList.contains(Position.ChuteUp))
                        .until(() -> !outList.contains(Position.ChuteUp))
                        .andThen(chuteDown.onlyIf(() -> outList.contains(Position.ChuteDown)));

        Command outputCommand =
                setFinalTarget.andThen(
                        followInPath
                                .andThen(clearOutPath)
                                .andThen(followOutPath)
                                .alongWith(chuteMover));

        outputCommand.addRequirements(this);
        return outputCommand;
    }

    public Command moveToTunablePosition() {
        return Commands.defer(
                () -> {
                    try {
                        var nextPos =
                                Position.valueOf(SuperstructureState.buttonTargetLogged.get());
                        nextPos.parent();
                        return moveToPosition(nextPos).asProxy();
                    } catch (IllegalArgumentException e) {
                        return Commands.runOnce(
                                () -> {
                                    Elastic.sendNotification(
                                            new Elastic.Notification(
                                                    Elastic.Notification.NotificationLevel.ERROR,
                                                    "Tunable Does Not Exist",
                                                    "The tunable in the superstructure button does not exist."));
                                });
                    }
                },
                Set.of());
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
                                    getChilderNodeInBranch(lastPosition, targetPostition).parent();
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
            WristSubsystem wristSubsystem,
            ChuteSubsystem chuteSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.chuteSubsystem = chuteSubsystem;

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
