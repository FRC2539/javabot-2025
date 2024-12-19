package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {
    private final LoggedDashboardChooser<Command> autoChooser;
    private Command previousAuto = Commands.none();
    private RobotConfig config; // PathPlanner robot configuration

    // *NEW
    private final Field2d m_trajectoryField = new Field2d();

    public Auto(CommandSwerveDrivetrain drivetrain) {
        setUpPathPlanner(drivetrain);
        autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
        SmartDashboard.putData("Auto Path", m_trajectoryField);
    }

    public void logAutoInformation() {
        if (previousAuto == autoChooser.get()) {
            return;
        }

        previousAuto = autoChooser.get();

        Command command = previousAuto;
        {
            try
            {
                var paths = PathPlannerAuto.getPathGroupFromAutoFile(command.getName()); // A list of all paths contained in this auto
                List<Pose2d> poses = new ArrayList<>(); // This will be a list of all points during the auto

                for (PathPlannerPath path : paths) { // For each path assigned, split into segments
                    List<PathPoint> points;
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        points = path.flipPath().getAllPathPoints();
                    } else {
                        points = path.getAllPathPoints();
                    }
                    for (PathPoint point : points) { // For each segment, split into points 
                        Pose2d newPose2d = new Pose2d(point.position, new Rotation2d());                    
                        poses.add(newPose2d);
                    }
                }

                // Generate a trajectory from the "poses" list. This is our entire path 
                // "config" is used for unit conversions; Reference Field2d Widget
                var m_trajectory = TrajectoryGenerator.generateTrajectory(poses, new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

                // Log the trajectory
                m_trajectoryField.getObject("traj").setTrajectory(m_trajectory);
                // Log the start and end positions

                Pose2d startingPose = paths.get(0).getStartingHolonomicPose().get();
                Pose2d endingPose = poses.get(poses.size()-1);
                endingPose = new Pose2d(endingPose.getX(), endingPose.getY(), paths.get(paths.size()-1).getGoalEndState().rotation());

                m_trajectoryField.getObject("start_and_end").setPoses(startingPose, endingPose);
              
            }
            catch (Exception e)
            {
                // Fallback in case the path is set to none, or the path file referenced does not exist
                e.printStackTrace();
                System.out.println("Pathplanner file not found! Skipping..."); 
                m_trajectoryField.getObject("traj").setPoses();
                m_trajectoryField.getObject("start_and_end").setPoses();
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
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            drivetrain::getRobotPose,
            drivetrain::resetPose,  
            drivetrain::getChassisSpeeds, 
            (speeds, feedforwards) -> drivetrain.setControl(drivetrain.driveWithFeedforwards(speeds, feedforwards)),
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ), 
            config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            drivetrain
        );

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            m_trajectoryField.getObject("Robot").setPose(pose);
        });
    }
}

