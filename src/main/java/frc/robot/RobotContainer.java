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
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.ControllerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState.Position;
import frc.robot.subsystems.arm.ArmPivotIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.WristIOSim;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.intake.FlipperIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.DummyPhotonCamera;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSimML;
import java.util.function.DoubleSupplier;

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

    public Auto auto = new Auto(drivetrain);
    public IntakeSubsystem intakeSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public ClimberSubsystem climberSubsystem;
    public ArmSubsystem armSubsystem;
    public Vision vision;

    public SuperstructureStateManager stateManager;

    public GripperSubsystem gripperSubsystem;
    // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private DoubleSupplier leftJoystickVelocityX;
    private DoubleSupplier leftJoystickVelocityY;

    public RobotContainer() {
        if (Robot.isReal()) {
            vision =
                    new Vision(
                            drivetrain::addVisionMeasurement,
                            new VisionIOLimelight(
                                    VisionConstants.camera0Name,
                                    () -> drivetrain.getRobotPose().getRotation()),
                            new DummyPhotonCamera(),
                            new VisionIOLimelight(
                                    VisionConstants.camera2Name,
                                    () -> drivetrain.getRobotPose().getRotation()));

            armSubsystem = new ArmSubsystem(new ArmPivotIOSim(), new WristIOSim());

            gripperSubsystem = new GripperSubsystem(new GripperIOSim());
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());

            intakeSubsystem = new IntakeSubsystem(new IntakeRollerIOSim(), new FlipperIOSim());

            climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
        } else {
            vision =
                    new Vision(
                            drivetrain::addVisionMeasurement,
                            new VisionIOPhotonVisionSim(
                                    VisionConstants.camera0Name,
                                    VisionConstants.robotToCamera0,
                                    drivetrain::getRobotPose),
                            new DummyPhotonCamera(),
                            new VisionIOPhotonVisionSimML(
                                    VisionConstants.camera2Name,
                                    VisionConstants.robotToCamera2,
                                    drivetrain::getRobotPose));

            gripperSubsystem = new GripperSubsystem(new GripperIOSim());
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
            armSubsystem = new ArmSubsystem(new ArmPivotIOSim(), new WristIOSim());
            intakeSubsystem = new IntakeSubsystem(new IntakeRollerIOSim(), new FlipperIOSim());
            climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
        }

        stateManager = new SuperstructureStateManager(elevatorSubsystem, armSubsystem);

        configureBindings();

        drivetrain.setUpPathPlanner();
        // Establish the "Trajectory Field" Field2d into the dashboard
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        leftJoystickVelocityX =
                () -> {
                    return GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond)
                            * -sps(deadband(leftDriveController.getYAxis().get(), 0.1));
                };
        leftJoystickVelocityY =
                () -> {
                    return -sps(deadband(leftDriveController.getXAxis().get(), 0.1))
                            * GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond);
                };
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically

                drivetrain.applyRequest(
                        () -> {
                            ChassisSpeeds driverDesiredSpeeds =
                                    new ChassisSpeeds(
                                            leftJoystickVelocityX.getAsDouble(),
                                            leftJoystickVelocityY.getAsDouble(),
                                            -sps(
                                                            deadband(
                                                                    rightDriveController
                                                                            .getXAxis()
                                                                            .get(),
                                                                    0.1))
                                                    * GlobalConstants.MAX_ROTATIONAL_SPEED.in(
                                                            RadiansPerSecond));
                            //     return drivetrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(
                            //             driverDesiredSpeeds);
                            return drivetrain.driveDriverRelative(driverDesiredSpeeds);
                        }));

        // drive.withVelocityX(-leftDriveController.getYAxis().get() *
        // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive forward with negative Y (forward)
        //     .withVelocityY(-leftDriveController.getXAxis().get() *
        // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive left with negative X (left)
        //     .withRotationalRate(-rightDriveController.getXAxis().get() *
        // GlobalConstants.MAX_ROTATIONAL_SPEED) // Drive counterclockwise with negative X (left)

        // operatorController.getA().whileTrue(drivetrain.applyRequest(() -> brake));
        // operatorController.getA().onTrue(new alignToTargetX(drivetrain, vision, 10, 0));

        // operatorController
        //         .getA()
        //         .onTrue(
        //                 new AlignToAngle(
        //                                 drivetrain,
        //                                 new Rotation2d(),
        //                                 true,
        //                                 leftJoystickVelocityX,
        //                                 leftJoystickVelocityY)
        //                         .andThen(
        //                                 new alignToTargetX(
        //                                         drivetrain, vision, 10, 0,
        // leftJoystickVelocityX)));

        // operatorController.getA().toggleOnTrue(alignToReef(9, 0));
        leftDriveController.getBottomThumb().whileTrue(alignToReef(9, 0));
        leftDriveController.getRightThumb().whileTrue(alignToReef(9, 0.4));
        leftDriveController.getLeftThumb().whileTrue(alignToReef(9, -0.4));
        // operatorController
        //         .getB()
        //         .whileTrue(
        //                 drivetrain.applyRequest(
        //                         () ->
        //                                 point.withModuleDirection(
        //                                         new Rotation2d(
        //                                                 -operatorController.getLeftYAxis().get(),
        //                                                 -operatorController
        //                                                         .getLeftXAxis()
        //                                                         .get()))));

        leftDriveController
                .getTrigger()
                .whileTrue(
                        new WheelRadiusCharacterization(
                                WheelRadiusCharacterization.Direction.CLOCKWISE, drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // operatorController
        //         .getBack()
        //         .and(operatorController.getY())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // operatorController
        //         .getBack()
        //         .and(operatorController.getX())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // operatorController
        //         .getStart()
        //         .and(operatorController.getY())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // operatorController
        //         .getStart()
        //         .and(operatorController.getX())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        // operatorController
        //         .getBack()
        //         .and(operatorController.getY())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // operatorController
        //         .getBack()
        //         .and(operatorController.getX())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // operatorController
        //         .getStart()
        //         .and(operatorController.getY())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // operatorController
        //         .getStart()
        //         .and(operatorController.getX())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // operatorController
        operatorController.getA().onTrue(stateManager.moveToPosition(Position.L4));
        operatorController.getB().onTrue(stateManager.moveToPosition(Position.L3));
        operatorController.getX().onTrue(stateManager.moveToPosition(Position.Source));
        operatorController.getY().onTrue(stateManager.moveToPosition(Position.Home));

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

        // return drivetrain.applyRequest(() ->
        //                 drivetrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(new
        // ChassisSpeeds(GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond), 0, 0))
        //         ).withTimeout(1).andThen(drivetrain.applyRequest(() ->
        //         drivetrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(new
        // ChassisSpeeds(-GlobalConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond), 0,
        // 0))).withTimeout(3));

    }

    public Command alignToReef(int tag, double offset) {
        Pose2d alignmentPose = VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d();
        return new AlignToReef(
                drivetrain,
                leftJoystickVelocityX,
                leftJoystickVelocityY,
                offset,
                alignmentPose,
                Rotation2d.kPi);
    }

    public boolean getVerticality() {
        return vision.isCoralVertical(0);
    }
}
