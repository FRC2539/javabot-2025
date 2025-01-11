// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.TestBase;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.ControllerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.WheelRadiusCharacterization;

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

    public final CommandSwerveDrivetrain driveTrain = TunerConstants.createDrivetrain();

    public Auto auto = new Auto(driveTrain);

    public TestBase TestBase = new TestBase();



    // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public RobotContainer() {
        configureBindings();


        driveTrain.setUpPathPlanner();
        // Establish the "Trajectory Field" Field2d into the dashboard
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // driveTrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
        //         driveTrain.applyRequest(
        //                 () -> {
        //                     ChassisSpeeds driverDesiredSpeeds =
        //                             new ChassisSpeeds(
        //                                     GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(
        //                                                     MetersPerSecond)
        //                                             * sps(
        //                                                     deadband(
        //                                                             leftDriveController
        //                                                                     .getYAxis()
        //                                                                     .get(),
        //                                                             0.1)),
        //                                     sps(deadband(leftDriveController.getXAxis().get(), 0.1))
        //                                             * GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(
        //                                                     MetersPerSecond),
        //                                     -sps(
        //                                                     deadband(
        //                                                             rightDriveController
        //                                                                     .getXAxis()
        //                                                                     .get(),
        //                                                             0.1))
        //                                             * GlobalConstants.MAX_ROTATIONAL_SPEED.in(
        //                                                     RadiansPerSecond));
        //                     return driveTrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(
        //                             driverDesiredSpeeds);
        //                     // return drivetrain.m_applyFieldSpeeds.withSpeeds(driverDesiredSpeeds);
        //                 }));

        // // drive.withVelocityX(-leftDriveController.getYAxis().get() *
        // // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive forward with negative Y (forward)
        // //     .withVelocityY(-leftDriveController.getXAxis().get() *
        // // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive left with negative X (left)
        // //     .withRotationalRate(-rightDriveController.getXAxis().get() *
        // // GlobalConstants.MAX_ROTATIONAL_SPEED) // Drive counterclockwise with negative X (left)

        // operatorController.getA().whileTrue(driveTrain.applyRequest(() -> brake));
        // operatorController
        //         .getB()
        //         .whileTrue(
        //                 driveTrain.applyRequest(
        //                         () ->
        //                                 point.withModuleDirection(
        //                                         new Rotation2d(
        //                                                 -operatorController.getLeftYAxis().get(),
        //                                                 -operatorController
        //                                                         .getLeftXAxis()
        //                                                         .get()))));

        // leftDriveController
        //         .getTrigger()
        //         .whileTrue(
        //                 new WheelRadiusCharacterization(
        //                         WheelRadiusCharacterization.Direction.CLOCKWISE, driveTrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //operatorController.getStart().whileTrue(TestBase.move().until(() -> TestBase.simulationGetBoolean()));
        TestBase.setDefaultCommand(TestBase.stop());
        operatorController.getRightBumper().whileTrue(TestBase.runSpeedsV2());
        TestBase.trigger2.whileTrue(TestBase.runSpeedsV2());
        

        // reset the field-centric heading on left bumper press
        // operatorController
        //         .getLeftBumper()
        //         .onTrue(driveTrain.runOnce(() -> driveTrain.seedFieldCentric()));
        // operatorController
        //         .getRightBumper()
        //         .onTrue(driveTrain.runOnce(() -> driveTrain.resetPose(new Pose2d())));
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
