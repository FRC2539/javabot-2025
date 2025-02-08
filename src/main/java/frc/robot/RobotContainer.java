// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.subsystems.TestBase;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

    private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(0);
    private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(1);
    private final LogitechController operatorController = new LogitechController(2);

    // public final CommandSwerveDrivetrain driveTrain = TunerConstants.createDrivetrain();

    // public Auto auto = new Auto(driveTrain);

    public TestBase TestBase = new TestBase();

    // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public RobotContainer() {
        configureBindings();

        // driveTrain.setUpPathPlanner();
        // Establish the "Trajectory Field" Field2d into the dashboard
    }

    private boolean shutdownOne = false;
    private boolean shutdownTwo = false;

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
        //                                     sps(deadband(leftDriveController.getXAxis().get(),
        // 0.1))
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
        //                     // return
        // drivetrain.m_applyFieldSpeeds.withSpeeds(driverDesiredSpeeds);
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
        // operatorController.getStart().whileTrue(TestBase.move().until(() ->
        // TestBase.simulationGetBoolean()));
        // TestBase.setDefaultCommand(TestBase.stop());
        // operatorController.getRightBumper().whileTrue(TestBase.runSpeedsV2());
        // TestBase.trigger2.whileTrue(TestBase.runSpeedsV2());

        TestBase.setDefaultCommand(
                TestBase.run(
                        () -> {
                            Logger.recordOutput("Testbase/Motor1", TestBase.motor1.get());
                            // Logger.recordOutput("Testbase/Motor2", TestBase.motor2.get());

                            Logger.recordOutput(
                                    "Testbase/Motor1Temp", TestBase.motor1.getMotorTemperature());
                            //     Logger.recordOutput(
                            //             "Testbase/Motor2Temp",
                            //
                            // TestBase.motor2.getDeviceTemp().refresh().getValueAsDouble());

                            //     Logger.recordOutput(
                            //             "Testbase/Motor2Cur",
                            //             TestBase.motor2
                            //                     .getStatorCurrent()
                            //                     .refresh()
                            //                     .getValueAsDouble());
                            Logger.recordOutput(
                                    "Testbase/Motor1Cur", TestBase.motor1.getOutputCurrent());

                            // motor 3 info
                            Logger.recordOutput("Testbase/Motor3", TestBase.motor3.get());
                            Logger.recordOutput(
                                    "Testbase/Motor3Temp",
                                    TestBase.motor3.getDeviceTemp().refresh().getValueAsDouble());
                            Logger.recordOutput(
                                    "Testbase/Motor3Cur",
                                    TestBase.motor3
                                            .getStatorCurrent()
                                            .refresh()
                                            .getValueAsDouble());

                            if (TestBase.motor1.getMotorTemperature() > 60) {
                                shutdownOne = true;
                            } else if (TestBase.motor1.getMotorTemperature() < 58) {
                                shutdownOne = false;
                            }
                            shutdownTwo = false;
                            // if (TestBase.motor2.getMotorTemperature() > 60) {
                            //     shutdownTwo = true;
                            // } else if (TestBase.motor2.getMotorTemperature() < 58) {
                            //     shutdownTwo = false;
                            // }
                            if (!shutdownOne) {
                                if (operatorController.getLeftBumper().getAsBoolean()) {
                                    TestBase.setMotor1Speed(
                                            TestBase.motor1SpeedNetworkNumber.get());
                                } else if (operatorController.getLeftTrigger().getAsBoolean()) {
                                    TestBase.setMotor1Speed(
                                            TestBase.motor1SpeedNetworkNumber.get() * -1);
                                } else {
                                    TestBase.setMotor1Speed(0);
                                }
                            } else {
                                TestBase.setMotor1Speed(0);
                            }

                            if (!shutdownTwo) {
                                if (operatorController.getRightBumper().getAsBoolean()) {
                                    TestBase.setMotor2Speed(
                                            TestBase.motor2SpeedNetworkNumber.get());
                                } else if (operatorController.getRightTrigger().getAsBoolean()) {
                                    TestBase.setMotor2Speed(
                                            TestBase.motor2SpeedNetworkNumber.get() * -1);
                                } else {
                                    TestBase.setMotor2Speed(0);
                                }
                            } else {
                                TestBase.setMotor2Speed(0);
                            }

                            if (true) {
                                if (operatorController.getY().getAsBoolean()) {
                                    TestBase.setMotor3Speed(
                                            TestBase.motor2SpeedNetworkNumber.get());
                                } else if (operatorController.getA().getAsBoolean()) {
                                    TestBase.setMotor3Speed(
                                            TestBase.motor2SpeedNetworkNumber.get() * -1);
                                } else {
                                    TestBase.setMotor3Speed(0);
                                }
                            } else {
                                TestBase.setMotor3Speed(0);
                            }
                        }));
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
        return Commands.none();
    }
}
