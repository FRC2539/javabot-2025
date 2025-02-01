package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
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
    public static double leftOffset = -1;
    public static double rightOffset = 1;

    public enum DriveLocation {
        None(0, 0, 0),
        SourceLeft(1, 13, 0),
        SourceRight(2, 12, 0),
        A(7, 18, leftOffset),
        B(7, 18, rightOffset),
        C(8, 17, leftOffset),
        D(8, 17, rightOffset),
        E(9, 22, leftOffset),
        F(9, 22, rightOffset),
        G(10, 21, leftOffset),
        H(10, 21, rightOffset),
        I(11, 20, leftOffset),
        J(11, 20, rightOffset),
        K(6, 19, leftOffset),
        L(6, 19, rightOffset),
        ;

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
        None(Position.None, 0),
        Home(Position.Home, 0),
        L1(Position.L1, 1),
        L2(Position.L2, 1),
        L3(Position.L3, 1),
        L4(Position.L4, 1),
        Source(Position.Source, -1);

        public Position position;
        public double armMotorSpeed;

        private ArmHeight(Position position, double armMotorSpeed) {
            this.position = position;
            this.armMotorSpeed = armMotorSpeed;
        }
    }

    // #146: Add a function that will register all triggers
    private double alignTimeout = 5;
    private double placeTimeout = 0.5;

    private void configureBindings() {
        NamedCommands.registerCommand(
                "wait", Commands.waitUntil(() -> (alignCommand == null && heightCommand == null)));

        EventTrigger alignTrigger = new EventTrigger("align");
        alignTrigger.onTrue(
                alignCommand =
                        Commands.runOnce(
                                        () -> {
                                            if (alignCommand != null) alignCommand.cancel();
                                        })
                                .alongWith(
                                        robotContainer
                                                .alignToReef(
                                                        targetLocation.getTagByTeam(),
                                                        targetLocation.offset)
                                                .withTimeout(alignTimeout)));

        EventTrigger armTrigger = new EventTrigger("arm");
        armTrigger.onTrue(
                heightCommand =
                        Commands.runOnce(
                                        () -> {
                                            if (heightCommand != null) heightCommand.cancel();
                                        })
                                .alongWith(
                                        robotContainer.stateManager.moveToPosition(
                                                targetHeight.position)));

        // Prep Arm Trigger
        EventTrigger prepArmTrigger = new EventTrigger("prepArm");
        Command prepArmCommand =
                Commands.defer(
                        () ->
                                robotContainer.stateManager.moveToPosition(
                                        targetHeight.position.parent),
                        Set.of(
                                robotContainer.armSubsystem,
                                robotContainer.elevatorSubsystem,
                                robotContainer.stateManager));
        prepArmTrigger.onTrue(prepArmCommand);

        EventTrigger placeTrigger = new EventTrigger("place");
        placeTrigger.onTrue(
                Commands.runOnce(
                        () -> {
                            if (alignCommand != null) alignCommand.cancel();
                            if (heightCommand != null) heightCommand.cancel();

                            alignCommand =
                                    robotContainer
                                            .alignToReef(
                                                    targetLocation.getTagByTeam(),
                                                    targetLocation.offset)
                                            .withTimeout(alignTimeout); // Ref: Align Trigger
                            heightCommand =
                                    robotContainer.stateManager.moveToPosition(
                                            targetHeight.position.parent); // Ref: Arm Trigger
                            alignCommand
                                    .alongWith(heightCommand)
                                    .until(() -> (alignCommand == null && heightCommand == null))
                                    .andThen(
                                            robotContainer
                                                    .stateManager
                                                    .moveToPosition(targetHeight.position)
                                                    .andThen(
                                                            robotContainer
                                                                    .gripperSubsystem
                                                                    .ejectSpin()
                                                                    .withTimeout(placeTimeout))
                                                    .andThen(
                                                            robotContainer.gripperSubsystem
                                                                    .setVoltage(0)));
                        }));

        // #region Aligns and Height
        for (DriveLocation location : DriveLocation.values()) {
            new EventTrigger("location ".concat(location.name()))
                    .onTrue(
                            Commands.runOnce(
                                    () -> {
                                        targetLocation = location;
                                    }));
        }

        for (ArmHeight height : ArmHeight.values()) {
            new EventTrigger("height ".concat(height.name()))
                    .onTrue(
                            Commands.runOnce(
                                    () -> {
                                        targetHeight = height;
                                    }));
        }
        // #endregion
    }
    ;
}
