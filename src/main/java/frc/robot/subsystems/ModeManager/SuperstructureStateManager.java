package frc.robot.subsystems.ModeManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
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
import org.littletonrobotics.junction.AutoLogOutput;
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
                new LoggedNetworkNumber("Elevator Height", 165);
        private static LoggedNetworkNumber armHeightLogged =
                new LoggedNetworkNumber("Arm Height", 0);
        private static LoggedNetworkNumber wristRotationLogged =
                new LoggedNetworkNumber("Wrist Rotation", 1.58);
        private static LoggedNetworkString parentLogged =
                new LoggedNetworkString("Superstructure Parent", "CenterZoneNull");
        private static LoggedNetworkString buttonTargetLogged =
                new LoggedNetworkString("Superstructure Button Target", "Tunable");

        @FunctionalInterface
        public interface StateChecker {
            boolean checksOut(
                    Position position, SuperstructureStateManager stateManager, boolean logExtra);

            default boolean checksOut(Position position, SuperstructureStateManager stateManager) {
                return checksOut(position, stateManager, false);
            }
        }

        public static final StateChecker FALSE = (p, s, e) -> false;
        public static final StateChecker TRUE = (p, s, e) -> true;
        public static final StateChecker DEFAULT =
                (p, s, e) -> {
                    // return (s.internalPosition == p);
                    double armPosition = s.armSubsystem.getPosition();
                    double wristPosition = s.wristSubsystem.getFlippedPosition();
                    double elevatorPosition = s.elevatorSubsystem.getPosition();

                    boolean armAtPosition = Math.abs(armPosition - p.armHeight()) < 0.1;
                    boolean wristAtPosition = Math.abs(wristPosition - p.wristRotation()) < 0.1;
                    boolean elevatorAtPosition =
                            Math.abs(elevatorPosition - p.elevatorHeight()) < 1.5;
                    if (e) {
                        Logger.recordOutput("/Superstructure/Arm/ArmAtPosition", armAtPosition);
                        Logger.recordOutput("/Superstructure/Arm/Setpoint", p.armHeight());
                        Logger.recordOutput("/Superstructure/Arm/Position", armPosition);
                        Logger.recordOutput(
                                "/Superstructure/Arm/Error", armPosition - p.armHeight());

                        Logger.recordOutput(
                                "/Superstructure/Wrist/WristAtPosition", wristAtPosition);
                        Logger.recordOutput("/Superstructure/Wrist/Setpoint", p.wristRotation());
                        Logger.recordOutput("/Superstructure/Wrist/Position", wristPosition);
                        Logger.recordOutput(
                                "/Superstructure/Wrist/Error", wristPosition - p.wristRotation());

                        Logger.recordOutput(
                                "/Superstructure/Elevator/ElevatorAtPosition", elevatorAtPosition);
                        Logger.recordOutput(
                                "/Superstructure/Elevator/Setpoint", p.elevatorHeight());
                        Logger.recordOutput("/Superstructure/Elevator/Position", elevatorPosition);
                        Logger.recordOutput(
                                "/Superstructure/Elevator/Error",
                                elevatorPosition - p.elevatorHeight());

                        Logger.recordOutput("/Superstructure/Checker", "DEFAULT");

                        Logger.recordOutput("Superstructure/Heartbeat", Timer.getTimestamp());
                    }
                    if (armAtPosition && wristAtPosition && elevatorAtPosition) {
                        return true;

                    } else return false;
                };
        public static final StateChecker AUTO = DEFAULT;

        // Any positions going lower than elevator 160 need to have Chute Up as a parent eventually.
        // (The handoff is the exception).
        // No arm positions should be negative unless they are for the handoff.
        // Wrist positions should be 1.58 unless the arm angle is positive. Then they can be
        // whatever.
        public static enum Position {
            Sussy(1, 1, 1, null),
            CenterZoneNull(1, 1, 1, null, TRUE, FALSE),
            Home(165, 0, 1.58, CenterZoneNull),
            ChuteDownPre(
                    165,
                    0,
                    1.58,
                    CenterZoneNull,
                    (a, s, e) -> s.chuteSubsystem.DOWN.getAsBoolean() || DEFAULT.checksOut(a, s, e),
                    (a, s, e) -> !s.chuteSubsystem.DOWN.getAsBoolean()),
            ChuteDown(
                    165,
                    0,
                    1.58,
                    ChuteDownPre,
                    (a, s, e) -> s.chuteSubsystem.DOWN.getAsBoolean(),
                    (a, s, e) -> !s.chuteSubsystem.DOWN.getAsBoolean()),
            ChuteDownNull(0, 0, 0, ChuteDown, TRUE, FALSE),
            ChuteUpPre(
                    165,
                    0,
                    1.58,
                    CenterZoneNull,
                    (a, s, e) -> s.chuteSubsystem.UP.getAsBoolean() || DEFAULT.checksOut(a, s, e),
                    (a, s, e) -> !s.chuteSubsystem.UP.getAsBoolean()),
            ChuteUp(
                    165,
                    0,
                    1.58,
                    ChuteUpPre,
                    (a, s, e) -> s.chuteSubsystem.UP.getAsBoolean(),
                    (a, s, e) -> !s.chuteSubsystem.UP.getAsBoolean()),
            ChuteUpNull(0, 0, 0, ChuteUp, TRUE, FALSE),
            HandoffPrep(165, -0.5, 1.58, ChuteDownNull),
            Handoff(140, -0.5, 1.58, HandoffPrep),
            PointUp(165, 2.05, 1.58, CenterZoneNull),
            UpZoneNull(0, 0, 0, PointUp, TRUE, FALSE),
            L1Prep(298.5, 2.05, -1.58, ChuteUpNull),
            L1(298.5, 2.05, -1.58, L1Prep),
            L2Prep(298.5, 2.05, -1.58, UpZoneNull),
            L2(298.5, 2.05, -1.58, L2Prep),
            L3Prep(298.5, 2.05, -1.58, UpZoneNull),
            L3(298.5, 2.05, -1.58, L3Prep),
            L4Prep(298.5, 2.05, -1.58, UpZoneNull),
            L4(298.5, 2.05, -1.58, L4Prep),
            Quick34(298.5, 2.05, -1.58, UpZoneNull),
            Quick23(298.5, 2.05, -1.58, UpZoneNull),
            L2AlgaePrep(298.5, 2.05, -1.58, UpZoneNull),
            L2Algae(298.5, 2.05, -1.58, L2AlgaePrep),
            L3AlgaePrep(298.5, 2.05, -1.58, UpZoneNull),
            L3Algae(298.5, 2.05, -1.58, L3AlgaePrep),
            NetAlgaePrep(298.5, 2.05, -1.58, UpZoneNull),
            NetAlgae(298.5, 2.05, -1.58, NetAlgaePrep),
            Source(165, 2.05, 0, PointUp),
            AlgaeHome(165, 0, 1.58, CenterZoneNull),
            Climb(165, 0, 1.58, ChuteUpNull),
            Processor(165, 0, 1.58, ChuteUpNull),
            IcecreamCoral(165, 0, 1.58, ChuteUpNull),
            IcecreamAlgae(165, 0, 1.58, ChuteUpNull),
            StartPrep(140, 0, -1.58, ChuteUpNull),
            Start(0, 0, -1.58, StartPrep),
            Tunable(165, 0, 1.58, CenterZoneNull, FALSE);

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
                    return SuperstructureState.elevatorHeightLogged
                            .get(); // ref: tunable variable armHeight
                }
                return elevatorHeight;
            }

            public double armHeight() {
                if (this == Position.Tunable) {
                    return SuperstructureState.armHeightLogged
                            .get(); // ref: tunable variable armHeight
                }
                return armHeight;
            }

            public double wristRotation() {
                if (this == Position.Tunable) {
                    return SuperstructureState.wristRotationLogged.get();
                    // ref: tunable variable wristRotation
                }
                return wristRotation;
            }

            public Position parent() {
                if (this == Position.Tunable) {
                    return Position.valueOf(parentLogged.get()); // ref: tunable variable parent
                }
                return parent;
            }
            // #endregion

        }
    }

    @AutoLogOutput
    public boolean isAtTargetAllegedly() {
        return SuperstructureState.DEFAULT.checksOut(targetPostition, this, true);
    }

    private SuperstructureState.Position targetPostition = SuperstructureState.Position.Start;
    private SuperstructureState.Position lastPosition = SuperstructureState.Position.Start;

    private List<SuperstructureState.Position> outList = new ArrayList<>();

    private enum CoralAlgaeMode {
        LeftCoral,
        RightCoral,
        Algae,
        ArmWrist;
    }

    private CoralAlgaeMode coralAlgaeMode = CoralAlgaeMode.LeftCoral;

    private SuperstructureState.Position lastRealPosition = Position.Home;

    private Pose2d lastScoringPose = Pose2d.kZero;

    @AutoLogOutput private boolean chuteCanMove = false;

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
            if (myPosition == SuperstructureState.Position.CenterZoneNull) {
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
        if (lastPosition.parent() == Position.CenterZoneNull && lastPosition.isRealPosition(this)) {
            chuteCanMove = true;
        }

        if (myPosition.isRealPosition(this)) {
            lastRealPosition = myPosition;
        }

        return elevatorSubsystem
                .setPosition(lastRealPosition.elevatorHeight())
                .alongWith(armSubsystem.setPosition(lastRealPosition.armHeight()))
                .alongWith(wristSubsystem.setPosition(lastRealPosition.wristRotation()));
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
                                return internalGoToPosition(Position.CenterZoneNull);
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
