package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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

    // *NEW
    private final Field2d m_trajectoryField = new Field2d();

    public Auto(CommandSwerveDrivetrain drivetrain) {
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

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    // Do whatever you want with the pose here
                    m_trajectoryField.getObject("Robot").setPose(pose);
                });
    }
}
