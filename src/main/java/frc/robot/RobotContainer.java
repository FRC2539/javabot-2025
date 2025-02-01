// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.commands.AlignToPiece;
import frc.robot.commands.AlignToReef;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.ControllerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState.Position;
import frc.robot.subsystems.arm.ArmPivotIOSim;
import frc.robot.subsystems.arm.ArmPivotIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.WristIONeo550;
import frc.robot.subsystems.arm.WristIOSim;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperIOFalcon;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.intake.FlipperIOSim;
import frc.robot.subsystems.intake.FlipperIOTalon;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSimML;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {
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

    private DoubleSupplier leftJoystickVelocityX;
    private DoubleSupplier leftJoystickVelocityY;
    private DoubleSupplier rightJoystickVelocityTheta;

    private Supplier<ChassisSpeeds> driverVelocitySupplier;

    public RobotContainer() {
        if (Robot.isReal()) {
            vision =
                    new Vision(
                            drivetrain::addVisionMeasurement,
                            new VisionIOLimelight(
                                    VisionConstants.camera0Name,
                                    () -> drivetrain.getRobotPose().getRotation()),
                            new VisionIOLimelight(
                                    VisionConstants.camera1Name,
                                    () -> drivetrain.getRobotPose().getRotation()),
                            new VisionIOLimelight(
                                    VisionConstants.camera2Name,
                                    () -> drivetrain.getRobotPose().getRotation()));
            gripperSubsystem = new GripperSubsystem(new GripperIOFalcon());
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFX());
            armSubsystem = new ArmSubsystem(new ArmPivotIOTalonFX(), new WristIONeo550());
            climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());

            intakeSubsystem = new IntakeSubsystem(new IntakeRollerTalonFX(), new FlipperIOTalon());
        } else {
            vision =
                    new Vision(
                            drivetrain::addVisionMeasurement,
                            new VisionIOPhotonVisionSim(
                                    VisionConstants.camera0Name,
                                    VisionConstants.robotToCamera0,
                                    drivetrain::getRobotPose),
                            new VisionIOPhotonVisionSim(
                                    VisionConstants.camera1Name,
                                    VisionConstants.robotToCamera1,
                                    drivetrain::getRobotPose),
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
        rightJoystickVelocityTheta =
                () -> {
                    return -sps(deadband(rightDriveController.getXAxis().get(), 0.1))
                            * GlobalConstants.MAX_ROTATIONAL_SPEED.in(RadiansPerSecond);
                };

        driverVelocitySupplier =
                () ->
                        new ChassisSpeeds(
                                leftJoystickVelocityX.getAsDouble(),
                                leftJoystickVelocityY.getAsDouble(),
                                rightJoystickVelocityTheta.getAsDouble());

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically

                drivetrain.applyRequest(
                        () -> {
                            //     return drivetrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(
                            //             driverDesiredSpeeds);
                            return drivetrain.driveDriverRelative(driverVelocitySupplier.get());
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
        // leftDriveController.getBottomThumb().whileTrue(alignToReef(9, 0));
        // leftDriveController.getRightThumb().whileTrue(alignToReef(9, 0.4));
        // leftDriveController.getLeftThumb().whileTrue(alignToReef(9, -0.4));
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

        // leftDriveController
        //         .getTrigger()
        //         .whileTrue(
        //                 new WheelRadiusCharacterization(
        //                         WheelRadiusCharacterization.Direction.CLOCKWISE, drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

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
        // operatorController.getA().onTrue(stateManager.moveToPosition(Position.L4));
        // operatorController.getB().onTrue(stateManager.moveToPosition(Position.L3));
        // operatorController.getX().onTrue(stateManager.moveToPosition(Position.Source));
        // operatorController.getY().onTrue(stateManager.moveToPosition(Position.Home));

        // reset the field-centric heading on left bumper press
        // operatorController
        //         .getLeftBumper()
        //         .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // operatorController
        //         .getRightBumper()
        //         .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

        // Operator Mode Setting
        operatorController.getLeftBumper().onTrue(stateManager.setLeftCoralMode());
        operatorController.getRightBumper().onTrue(stateManager.setRightCoralMode());
        operatorController.getRightTrigger().onTrue(stateManager.setAlgaeMode());
        operatorController.getLeftJoystick().toggleOnTrue(Commands.idle()); // L3 Rainbow
        operatorController.getLeftTrigger().whileTrue(Commands.idle()); // L2 Station Lights

        // Coral Mode Bindings
        final Trigger CORAL = stateManager.LEFT_CORAL.or(stateManager.RIGHT_CORAL);
        final Trigger ALGAE = stateManager.ALGAE;
        CORAL.and(operatorController.getY()).onTrue(stateManager.moveToPosition(Position.L4));
        CORAL.and(operatorController.getX()).onTrue(stateManager.moveToPosition(Position.L3));
        CORAL.and(operatorController.getB()).onTrue(stateManager.moveToPosition(Position.L2));
        CORAL.and(operatorController.getA()).onTrue(stateManager.moveToPosition(Position.L1));
        CORAL.and(operatorController.getStart())
                .onTrue(stateManager.moveToPosition(Position.Source));
        CORAL.and(operatorController.getDPadDown())
                .onTrue(stateManager.moveToPosition(Position.Home));
        CORAL.and(operatorController.getDPadUp())
                .onTrue(stateManager.moveToPosition(Position.Handoff));
        CORAL.and(operatorController.getBack()).onTrue(Commands.none());

        ALGAE.and(operatorController.getY()).onTrue(stateManager.moveToPosition(Position.L4Algae));
        ALGAE.and(operatorController.getX()).onTrue(stateManager.moveToPosition(Position.L3Algae));
        ALGAE.and(operatorController.getB()).onTrue(stateManager.moveToPosition(Position.L2Algae));
        ALGAE.and(operatorController.getA()).onTrue(stateManager.moveToPosition(Position.L1Algae));
        ALGAE.and(operatorController.getStart())
                .onTrue(stateManager.moveToPosition(Position.Icecream));
        ALGAE.and(operatorController.getDPadDown())
                .onTrue(stateManager.moveToPosition(Position.Home));
        ALGAE.and(operatorController.getDPadUp())
                .onTrue(stateManager.moveToPosition(Position.Handoff));
        ALGAE.and(operatorController.getDPadDownLeft())
                .onTrue(stateManager.moveToPosition(Position.Quick34));
        ALGAE.and(operatorController.getDPadRight())
                .onTrue(stateManager.moveToPosition(Position.Quick23));
        ALGAE.and(operatorController.getBack()).onTrue(Commands.none());

        // Driver Align Bindings, for a different/later day
        // CORAL.and(leftDriveController.getTrigger()).whileTrue(alignToReef(9, 0));

        // Climb Bindings
        leftDriveController.getLeftThumb().whileTrue(climberSubsystem.moveClimberDown());
        leftDriveController.getRightThumb().whileTrue(climberSubsystem.moveClimberUp());

        // Intake Bindings
        rightDriveController.getLeftThumb().whileTrue(intakeSubsystem.intake());
        rightDriveController.getRightThumb().whileTrue(intakeSubsystem.eject());

        CORAL.and(rightDriveController.getBottomThumb())
                .whileTrue(gripperSubsystem.intakeSpinCoral());
        CORAL.and(rightDriveController.getTrigger()).whileTrue(gripperSubsystem.ejectSpinCoral());

        ALGAE.and(rightDriveController.getBottomThumb())
                .whileTrue(gripperSubsystem.intakeSpinAlgae());
        ALGAE.and(rightDriveController.getTrigger()).whileTrue(gripperSubsystem.ejectSpinAlgae());

        // Technical Bindings
        rightDriveController
                .getLeftTopLeft()
                .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
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

    public Command alignToReef(int tag, double offset) {
        Pose2d alignmentPose = VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d();
        return new AlignToReef(
                drivetrain,
                leftJoystickVelocityX,
                leftJoystickVelocityY,
                offset,
                alignmentPose,
                Rotation2d.kPi); // Skibidi
    }

    public Command alignToPiece() {
        Supplier<Pose2d> piecePositionSupplier = () -> new Pose2d(9.2, 4.15, Rotation2d.kZero);
        return new AlignToPiece(
                drivetrain, driverVelocitySupplier, 0, piecePositionSupplier, Rotation2d.kZero);
    }

    public boolean getVerticality() {
        return vision.isCoralVertical(0);
    }
}
