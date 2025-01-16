// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.ControllerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.WheelRadiusCharacterization;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {
    private double MaxSpeed =
            GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(
                    MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            GlobalConstants.MAX_ROTATIONAL_SPEED.in(
                    RadiansPerSecond); // kMaxAngularRate desired top rotational speed

    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private ElevatorSubsystem elevatorSubsystem;

    public Auto auto = new Auto(drivetrain);

    public Vision vision;
    // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public RobotContainer() {
        if (Robot.isReal()) {
                vision =
                    new Vision(
                            drivetrain::addVisionMeasurement,
                            new VisionIOLimelight(
                                    VisionConstants.camera0Name,
                                    () -> drivetrain.getRobotPose().getRotation()));
                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFX());
        
        
        } else {
                vision =
                    new Vision(
                            drivetrain::addVisionMeasurement,
                            new VisionIOPhotonVisionSim(
                                    VisionConstants.camera0Name,
                                    VisionConstants.robotToCamera0,
                                    drivetrain::getRobotPose));
                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
        }

        configureBindings();

        drivetrain.setUpPathPlanner();
        // Establish the "Trajectory Field" Field2d into the dashboard
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> {
                            ChassisSpeeds driverDesiredSpeeds =
                                    new ChassisSpeeds(
                                            GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(
                                                            MetersPerSecond)
                                                    * -sps(
                                                            deadband(
                                                                    leftDriveController
                                                                            .getYAxis()
                                                                            .get(),
                                                                    0.1)),
                                            -sps(
                                                            deadband(
                                                                    leftDriveController
                                                                            .getXAxis()
                                                                            .get(),
                                                                    0.1))
                                                    * GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(
                                                            MetersPerSecond),
                                            -sps(
                                                            deadband(
                                                                    rightDriveController
                                                                            .getXAxis()
                                                                            .get(),
                                                                    0.1))
                                                    * GlobalConstants.MAX_ROTATIONAL_SPEED.in(
                                                            RadiansPerSecond));
                            //     return drivetrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(
                            //     driverDesiredSpeeds);
                            return drivetrain.m_applyFieldSpeeds.withSpeeds(driverDesiredSpeeds);
                        }));

        // drive.withVelocityX(-leftDriveController.getYAxis().get() *
        // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive forward with negative Y (forward)
        //     .withVelocityY(-leftDriveController.getXAxis().get() *
        // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive left with negative X (left)
        //     .withRotationalRate(-rightDriveController.getXAxis().get() *
        // GlobalConstants.MAX_ROTATIONAL_SPEED) // Drive counterclockwise with negative X (left)

        operatorController.getA().whileTrue(drivetrain.applyRequest(() -> brake));
        operatorController
                .getB()
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        point.withModuleDirection(
                                                new Rotation2d(
                                                        -operatorController.getLeftYAxis().get(),
                                                        -operatorController
                                                                .getLeftXAxis()
                                                                .get()))));

        leftDriveController
                .getTrigger()
                .whileTrue(
                        new WheelRadiusCharacterization(
                                WheelRadiusCharacterization.Direction.CLOCKWISE, drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        operatorController
                .getBack()
                .and(operatorController.getY())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        operatorController
                .getBack()
                .and(operatorController.getX())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        operatorController
                .getStart()
                .and(operatorController.getY())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        operatorController
                .getStart()
                .and(operatorController.getX())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        operatorController
                .getLeftBumper()
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        operatorController
                .getRightBumper()
                .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));
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
        return auto.getAuto();
    }
}
