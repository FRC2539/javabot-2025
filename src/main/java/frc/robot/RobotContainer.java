// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.GlobalConstants.ControllerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LoggedDashboardChooser<Command> autoChooser;

    // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Field2d m_trajectoryField = new Field2d(); // *NEW
    public RobotContainer() {
        configureBindings();

        drivetrain.setUpPathPlanner();
        SmartDashboard.putData("Trajectory Field", m_trajectoryField); // Establish the "Trajectory Field" Field2d into the dashboard
        var tempAutoChooser = AutoBuilder.buildAutoChooser();
        tempAutoChooser.onChange((Command command) -> {
            try
            {
                var paths = PathPlannerAuto.getPathGroupFromAutoFile(command.getName()); // A list of all paths contained in this auto
                List<Pose2d> poses = new ArrayList<>(); // This will be a list of all points during the auto

                for (PathPlannerPath path : paths) { // For each path assigned, split into segments
                    List<PathPoint> points = path.getAllPathPoints(); 
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
                m_trajectoryField.getObject("start_and_end").setPoses(poses.get(0), poses.get(poses.size() -1));
            }
            catch (Exception e)
            {
                // Fallback in case the path is set to none, or the path file referenced does not exist
                e.printStackTrace();
                System.out.println("Pathplanner file not found! Skipping..."); 
                m_trajectoryField.getObject("traj").setPoses();
                m_trajectoryField.getObject("start_and_end").setPoses();
            }

        });
        SmartDashboard.putData(tempAutoChooser);
        
        autoChooser = new LoggedDashboardChooser<>("Auto Routine", tempAutoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () -> {
                    ChassisSpeeds driverDesiredSpeeds = new ChassisSpeeds(
                        sps(deadband(leftDriveController.getYAxis().get(), 0.1)) * GlobalConstants.MAX_TRANSLATIONAL_SPEED,
                        -sps(deadband(leftDriveController.getXAxis().get(),0.1)) * GlobalConstants.MAX_TRANSLATIONAL_SPEED,
                        -sps(deadband(rightDriveController.getXAxis().get(),0.1)) * GlobalConstants.MAX_ROTATIONAL_SPEED
                    );
                    return drivetrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(driverDesiredSpeeds);
                    // return drivetrain.m_applyFieldSpeeds.withSpeeds(new ChassisSpeeds(3,1,0));
                }
            )
        );

        // drive.withVelocityX(-leftDriveController.getYAxis().get() * GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive forward with negative Y (forward)
                //     .withVelocityY(-leftDriveController.getXAxis().get() * GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive left with negative X (left)
                //     .withRotationalRate(-rightDriveController.getXAxis().get() * GlobalConstants.MAX_ROTATIONAL_SPEED) // Drive counterclockwise with negative X (left)

        operatorController.getA().whileTrue(drivetrain.applyRequest(() -> brake));
        operatorController.getB().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-operatorController.getLeftYAxis().get(), -operatorController.getLeftXAxis().get()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        operatorController.getBack().and(operatorController.getY()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        operatorController.getBack().and(operatorController.getX()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        operatorController.getStart().and(operatorController.getY()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        operatorController.getStart().and(operatorController.getX()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        operatorController.getLeftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        operatorController.getRightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d())));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private double deadband(double value, double deadband) {
        if (value <= deadband && -deadband <= value) {
            return 0;
        }

        return value;
    }

    private double sps(double value) {
        return value * Math.abs(value);
    }

    public Command getAutonomousCommand() {
        
        return autoChooser.get();
        
    }
}
