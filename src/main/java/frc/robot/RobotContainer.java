// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public RobotContainer() {
        configureBindings();
    }

    public ChassisSpeeds getDesiredChassisSpeeds()
    {
        return new ChassisSpeeds(-leftDriveController.getYAxis().getAsDouble() * GlobalConstants.MAX_TRANSLATIONAL_SPEED, -leftDriveController.getXAxis().get() * GlobalConstants.MAX_TRANSLATIONAL_SPEED, -rightDriveController.getXAxis().get() * GlobalConstants.MAX_ROTATIONAL_SPEED);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.driveRobotRelative(-leftDriveController.getYAxis().get() * GlobalConstants.MAX_TRANSLATIONAL_SPEED, -leftDriveController.getXAxis().get() * GlobalConstants.MAX_TRANSLATIONAL_SPEED, -rightDriveController.getXAxis().get() * GlobalConstants.MAX_ROTATIONAL_SPEED);
                // drive.withVelocityX(-leftDriveController.getYAxis().get() * GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive forward with negative Y (forward)
                //     .withVelocityY(-leftDriveController.getXAxis().get() * GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive left with negative X (left)
                //     .withRotationalRate(-rightDriveController.getXAxis().get() * GlobalConstants.MAX_ROTATIONAL_SPEED) // Drive counterclockwise with negative X (left)
            )
        );

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

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
