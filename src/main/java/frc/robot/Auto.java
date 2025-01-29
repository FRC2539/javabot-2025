package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.TriggerEvent;
import com.pathplanner.lib.path.EventMarker;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState.Position;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Auto {
    private final LoggedDashboardChooser<Command> autoChooser;
    private Command previousAuto = Commands.none();
    private Alliance previousAlliance = Alliance.Blue;
    private RobotConfig config; // PathPlanner robot configuration

    // #146
    private RobotContainer robotContainer;
    private Location targetLocation = Location.None;
    private Height targetHeight = Height.None;

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

        configureBindings();

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    // Do whatever you want with the pose here
                    m_trajectoryField.getObject("Robot").setPose(pose);
                });
    }

    public enum Location {
        None,
        SourceLeft,
        SourceRight,
        A, B,
        C, D,
        E, F,
        G, H,
        I, J,
        K, L,
    } 
    public enum Height {
        None(Position.None),
        Home(Position.Home),
        L1(Position.L1),
        L2(Position.L2),
        L3(Position.L3),
        L4(Position.L4),
        ;

        public Position position;
        private Height(Position position) {
            this.position = position;
        }
    }

    // #146: Add a function that will register all triggers
    void configureBindings() {
        EventTrigger placeTrigger = new EventTrigger("place");
        placeTrigger.onTrue(
            Commands.run(() -> {
                // Ref: Align Trigger
            })
            .alongWith(Commands.run(() -> {
                robotContainer.stateManager.moveToPosition(targetHeight.position); // Ref: Arm Trigger
            }))
            .andThen() // Score
        );
        EventTrigger alignTrigger = new EventTrigger("align");
        alignTrigger.onTrue(
            Commands.run(() -> {
                // Take control of swerve and auto-align to selected location
            }).andThen()
        );
        EventTrigger armTrigger = new EventTrigger("arm");
        armTrigger.onTrue(
            Commands.runOnce(() -> {
                  robotContainer.stateManager.moveToPosition(targetHeight.position);
            })
        );

        //#region Aligns and Heights
        EventTrigger align_None = new EventTrigger("set location None"); align_None.onTrue(Commands.runOnce(() -> { targetLocation = Location.None; }));
        EventTrigger align_A = new EventTrigger("set location A"); align_A.onTrue(Commands.runOnce(() -> { targetLocation = Location.A; }));
        EventTrigger align_B = new EventTrigger("set location B"); align_B.onTrue(Commands.runOnce(() -> { targetLocation = Location.B; }));
        EventTrigger align_C = new EventTrigger("set location C"); align_C.onTrue(Commands.runOnce(() -> { targetLocation = Location.C; }));
        EventTrigger align_D = new EventTrigger("set location D"); align_D.onTrue(Commands.runOnce(() -> { targetLocation = Location.D; }));
        EventTrigger align_E = new EventTrigger("set location E"); align_E.onTrue(Commands.runOnce(() -> { targetLocation = Location.E; }));
        EventTrigger align_F = new EventTrigger("set location F"); align_F.onTrue(Commands.runOnce(() -> { targetLocation = Location.F; }));
        EventTrigger align_G = new EventTrigger("set location G"); align_G.onTrue(Commands.runOnce(() -> { targetLocation = Location.G; }));
        EventTrigger align_H = new EventTrigger("set location H"); align_H.onTrue(Commands.runOnce(() -> { targetLocation = Location.H; }));
        EventTrigger align_I = new EventTrigger("set location I"); align_I.onTrue(Commands.runOnce(() -> { targetLocation = Location.I; }));
        EventTrigger align_J = new EventTrigger("set location J"); align_J.onTrue(Commands.runOnce(() -> { targetLocation = Location.J; }));
        EventTrigger align_K = new EventTrigger("set location K"); align_K.onTrue(Commands.runOnce(() -> { targetLocation = Location.K; }));
        EventTrigger align_L = new EventTrigger("set location L"); align_L.onTrue(Commands.runOnce(() -> { targetLocation = Location.L; }));

        EventTrigger arm_None = new EventTrigger("set height None"); arm_None.onTrue(Commands.runOnce(() -> { targetHeight = Height.None; }));
        EventTrigger arm_1 = new EventTrigger("set height L1"); arm_1.onTrue(Commands.runOnce(() -> { targetHeight = Height.L1; }));
        EventTrigger arm_2 = new EventTrigger("set height L2"); arm_2.onTrue(Commands.runOnce(() -> { targetHeight = Height.L2; }));
        EventTrigger arm_3 = new EventTrigger("set height L3"); arm_3.onTrue(Commands.runOnce(() -> { targetHeight = Height.L3; }));
        EventTrigger arm_4 = new EventTrigger("set height L4"); arm_4.onTrue(Commands.runOnce(() -> { targetHeight = Height.L4; }));
        //#endregion
    };



}
