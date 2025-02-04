package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState.Position;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Auto {
    private final LoggedDashboardChooser<Command> autoChooser;
    private Command previousAuto = Commands.none();
    private Alliance previousAlliance = Alliance.Blue;
    private RobotConfig config; // PathPlanner robot configuration

    // #146
    private RobotContainer robotContainer;
    private DriveLocation targetLocation = DriveLocation.None;
    private ArmHeight targetHeight = ArmHeight.None;
    private Command alignCommand;
    private Command heightCommand;
    /* private void setTargetHeight(ArmHeight targetHeight)
    {
        this.targetHeight = targetHeight;
        robotContainer.stateManager.moveToPosition(targetHeight.position.parent);
    } */

    // *NEW
    private final Field2d m_trajectoryField = new Field2d();

    public Auto(CommandSwerveDrivetrain drivetrain, RobotContainer robotContainer) {
        setUpPathPlanner(drivetrain);
        autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
        SmartDashboard.putData("Auto Path", m_trajectoryField);
    }

    public void logAutoInformation() {
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Command currentCommand = autoChooser.get();

        if (previousAuto != currentCommand
                || (!DriverStation.isEnabled() && previousAlliance != currentAlliance)) {
            previousAlliance = currentAlliance;

            previousAuto = currentCommand;

            Command command = previousAuto;
            Optional<Alliance> alliance = DriverStation.getAlliance();
            {
                try {
                    var paths = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());

                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        for (int i = 0; i < paths.size(); i++) {
                            paths.set(i, paths.get(i).flipPath());
                        }
                    }

                    var trajectories = new PathPlannerTrajectory[paths.size()];

                    for (int i = 0; i < trajectories.length; i++) {
                        trajectories[i] = paths.get(i).getIdealTrajectory(config).get();
                    }

                    // A list of all paths contained in this auto
                    List<Pose2d> poses =
                            new ArrayList<>(); // This will be a list of all points during the auto

                    for (var path : trajectories) { // For each path assigned, split into segments
                        for (var point : path.getStates()) { // For each segment, split into points
                            poses.add(point.pose);
                        }
                    }

                    // Generate a trajectory from the "poses" list. This is our entire path
                    // "config" is used for unit conversions; Reference Field2d Widget
                    var m_trajectory =
                            TrajectoryGenerator.generateTrajectory(
                                    poses,
                                    new TrajectoryConfig(
                                            Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

                    // Log the trajectory
                    m_trajectoryField.getObject("traj").setTrajectory(m_trajectory);
                    // Log the start and end positions

                    Pose2d startingPose = trajectories[0].getInitialPose();
                    Pose2d endingPose = trajectories[trajectories.length - 1].getEndState().pose;

                    m_trajectoryField.getObject("start_and_end").setPoses(startingPose, endingPose);

                    System.out.println("Pathplanner auto successfully shared.");

                } catch (Exception e) {
                    // Fallback in case the path is set to none, or the path file referenced does
                    // not exist
                    e.printStackTrace();
                    System.out.println("Pathplanner file not found! Skipping...");
                    m_trajectoryField.getObject("traj").setPoses();
                    m_trajectoryField.getObject("start_and_end").setPoses();
                }
            }
        }
    }

    public Command getAuto() {
        if (AutoBuilder.isConfigured()) {
            return autoChooser.get();
        } else {
            return Commands.none();
        }
    }

    public void setUpPathPlanner(CommandSwerveDrivetrain drivetrain) {
        config = GlobalConstants.getRobotConfigPathplanner();

        AutoBuilder.configure(
                drivetrain::getRobotPose,
                drivetrain::resetPose,
                drivetrain::getChassisSpeeds,
                (speeds, feedforwards) ->
                        drivetrain.setControl(drivetrain.driveRobotRelative(speeds, feedforwards)),
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                        // following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                        ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain);

        configureBindings(); // #146

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    // Do whatever you want with the pose here
                    m_trajectoryField.getObject("Robot").setPose(pose);
                });
    }

    // #146 Constants
    public static double leftOffset = -0.2;
    public static double rightOffset = 0.2;
    public static double centerOffset = 0;

    public enum DriveLocation {
        None(0, 0, 0),
        SourceLeft(1, 13, 0),
        SourceRight(2, 12, 0),
        A(7, 18, leftOffset),
        AB(7, 18, centerOffset),
        B(7, 18, rightOffset),
        C(8, 17, leftOffset),
        CD(8, 17, centerOffset),
        D(8, 17, rightOffset),
        E(9, 22, leftOffset),
        EF(9, 22, centerOffset),
        F(9, 22, rightOffset),
        G(10, 21, leftOffset),
        GH(10, 21, centerOffset),
        H(10, 21, rightOffset),
        I(11, 20, leftOffset),
        JH(11, 20, centerOffset),
        J(11, 20, rightOffset),
        K(6, 19, leftOffset),
        KL(6, 19, centerOffset),
        L(6, 19, rightOffset);

        public int tagRed;
        public int tagBlue;
        public double offset;

        private DriveLocation(int tagRed, int tagBlue, double offset) {
            this.tagRed = tagRed;
            this.tagBlue = tagBlue;
            this.offset = offset;
        }

        public int getTagByTeam() {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) return tagRed;
                if (ally.get() == Alliance.Blue) return tagBlue;
            }

            return tagRed;
        }
    }

    public enum ArmHeight {
        None(Position.None, Position.None, 0),
        Home(Position.Home, Position.Home, 0),
        L1(Position.L1, Position.L1Prep, 1),
        L2(Position.L2, Position.L2Prep, 1),
        L3(Position.L3, Position.L3Prep, 1),
        L4(Position.L4, Position.L4Prep, 1),
        Source(Position.Source, Position.SourcePrep, -1);

        public Position position;
        public double armMotorSpeed;
        public Position prep;

        private ArmHeight(Position position, Position prep, double armMotorSpeed) {
            this.position = position;
            this.prep = prep;
            this.armMotorSpeed = armMotorSpeed;
        }
    }

    // #146: Add a function that will register all triggers
    private double alignTimeout = 5;
    private double placeTimeout = 0.5;

    private void configureBindings() {
        NamedCommands.registerCommand(
                "wait", Commands.waitUntil(() -> armInPlace() && robotInPlace()));

        Command alignCommand =
                Commands.defer(
                        () ->
                                robotContainer.alignAndDriveToReef(
                                        targetLocation.getTagByTeam(), targetLocation.offset),
                        Set.of(robotContainer.drivetrain));
        NamedCommands.registerCommand("align", alignCommand);

        Command armCommand =
                Commands.defer(
                        () -> robotContainer.stateManager.moveToPosition(targetHeight.position),
                        Set.of(
                                robotContainer.armSubsystem,
                                robotContainer.elevatorSubsystem,
                                robotContainer.stateManager));
        NamedCommands.registerCommand("arm", armCommand.asProxy());

        Command prepArmCommand =
                Commands.defer(
                        () -> robotContainer.stateManager.moveToPosition(targetHeight.prep),
                        Set.of(
                                robotContainer.armSubsystem,
                                robotContainer.elevatorSubsystem,
                                robotContainer.stateManager));
        NamedCommands.registerCommand("preparm", prepArmCommand.asProxy());

        Command scoreCommand =
                robotContainer.gripperSubsystem.ejectSpinCoral().withTimeout(placeTimeout);
        NamedCommands.registerCommand("score", prepArmCommand.asProxy());

        // spotless:off
        Command placeCommand =
                prepArmCommand.asProxy()
                .until(() -> robotInPlace()) // Wait until arm and align are in position
                .andThen(
                    (
                        Commands.waitUntil(() -> armInPlace())
                        .andThen(scoreCommand.asProxy())
                    )
                    .deadlineFor(armCommand.asProxy())
                )
                .deadlineFor(alignCommand);

        NamedCommands.registerCommand("score", placeCommand);
        // spotless:on

        Command intakeCommand = robotContainer.intakeSubsystem.intake();
        NamedCommands.registerCommand("intake", intakeCommand.asProxy());

        // #region Auto create locations and heights
        for (DriveLocation location : DriveLocation.values()) {
            NamedCommands.registerCommand(
                    "location ".concat(location.name()),
                    Commands.runOnce(
                            () -> {
                                targetLocation = location;
                            }));
        }

        for (ArmHeight height : ArmHeight.values()) {
            NamedCommands.registerCommand(
                    "height ".concat(height.name()),
                    Commands.runOnce(
                            () -> {
                                targetHeight = height;
                            }));
        }
        // #endregion
    }

    private boolean armInPlace() {
        return SuperstructureState.DEFAULT.isAtTarget(targetHeight.position, robotContainer.stateManager);
    }

    private boolean robotInPlace() {
        Pose2d alignmentPose = VisionConstants.aprilTagLayout.getTagPose(targetLocation.getTagByTeam()).get().toPose2d().plus(new Transform2d(0, targetLocation.offset, Rotation2d.kZero));
        Pose2d currentPose = robotContainer.drivetrain.getRobotPose();
        Pose2d relativePos = alignmentPose.relativeTo(currentPose);
        return (Math.abs(relativePos.getX()) < 0.1) && (Math.abs(relativePos.getY()) < 0.1) && (Math.abs(relativePos.getRotation().getRadians()) < 0.1);  
    }
}
