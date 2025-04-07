// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.commands.AlignAndDriveToReef;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.constants.AligningConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.ControllerConstants;
import frc.robot.constants.GripperConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ModeManager.ModeManager;
import frc.robot.subsystems.ModeManager.ModeManager.Position;
import frc.robot.subsystems.ModeManager.ModeManager.ScoringMode;
import frc.robot.subsystems.arm.ArmPivotIOSim;
import frc.robot.subsystems.arm.ArmPivotIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberHeadIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperIONeo550;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.DummyPhotonCamera;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSimML;
import java.util.Set;
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

    public Auto auto; // #146: Pass in RobotContainer
    // public IntakeSubsystem intakeSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public ClimberSubsystem climberSubsystem;
    public ArmSubsystem armSubsystem;

    public Vision vision;
    public LightsSubsystem lights;

    public ModeManager modeManager;
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
                            new DummyPhotonCamera());
            gripperSubsystem =
                    new GripperSubsystem(new GripperIONeo550()); // new GripperIOFalcon());
            elevatorSubsystem =
                    new ElevatorSubsystem(new ElevatorIOTalonFX()); // new ElevatorIOTalonFX());
            armSubsystem = new ArmSubsystem(new ArmPivotIOTalonFX());
            climberSubsystem =
                    new ClimberSubsystem(
                            new ClimberIOTalonFX(),
                            new ClimberHeadIOSim()); // new ClimberIOTalonFX(), new
            // ClimberHeadIONeo550());
            lights = new LightsSubsystem();

            //     intakeSubsystem = new IntakeSubsystem(new IntakeRollerIOSim(), new
            // FlipperIOSim());
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
            armSubsystem = new ArmSubsystem(new ArmPivotIOSim());
            //     intakeSubsystem = new IntakeSubsystem(new IntakeRollerIOSim(), new
            // FlipperIOSim());
            climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX(), new ClimberHeadIOSim());
            lights = new LightsSubsystem();
        }

        modeManager = new ModeManager(elevatorSubsystem, armSubsystem);
        auto = new Auto(drivetrain, this);

        configureBindings();
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
                            // return drivetrain.m_applyFieldSpeedsOrbit.withChassisSpeeds(
                            // driverDesiredSpeeds);
                            return drivetrain.driveDriverRelative(driverVelocitySupplier.get());
                        }));

        SmartDashboard.putData(
                drivetrain
                        .sysIdDynamic(Direction.kForward)
                        .withName("Swerve SysId Dynamic Forward"));
        SmartDashboard.putData(
                drivetrain
                        .sysIdDynamic(Direction.kReverse)
                        .withName("Swerve SysId Dynamic Reverse"));
        SmartDashboard.putData(
                drivetrain
                        .sysIdQuasistatic(Direction.kForward)
                        .withName("Swerve SysId Quasistatic Forward"));
        SmartDashboard.putData(
                drivetrain
                        .sysIdQuasistatic(Direction.kReverse)
                        .withName("Swerve SysId Quasistatic Reverse"));

        SmartDashboard.putData(
                drivetrain.sysIdRotationMode().withName("Swerve SysId Rotation Mode"));
        SmartDashboard.putData(drivetrain.sysIdSteerMode().withName("Swerve SysId Steer Mode"));
        SmartDashboard.putData(
                drivetrain.sysIdTranslationMode().withName("Swerve SysId Translation Mode"));

        SmartDashboard.putData(
                new WheelRadiusCharacterization(
                                WheelRadiusCharacterization.Direction.CLOCKWISE, drivetrain)
                        .withName("Wheel Radius Characterization Command"));

        SmartDashboard.putData(
                elevatorSubsystem
                        .runDynamicElevatorSysId(Direction.kForward)
                        .withName("Elevator SysId Dynamic Forward"));
        SmartDashboard.putData(
                elevatorSubsystem
                        .runDynamicElevatorSysId(Direction.kReverse)
                        .withName("Elevator SysId Dynamic Reverse"));
        SmartDashboard.putData(
                elevatorSubsystem
                        .runQStaticElevatorSysId(Direction.kForward)
                        .withName("Elevator SysId Quasistatic Forward"));
        SmartDashboard.putData(
                elevatorSubsystem
                        .runQStaticElevatorSysId(Direction.kReverse)
                        .withName("Elevator SysId Quasistatic Reverse"));

        SmartDashboard.putData(elevatorSubsystem);

        SmartDashboard.putData(
                armSubsystem
                        .runDynamicArmSysId(Direction.kForward)
                        .withName("Arm SysId Dynamic Forward"));
        SmartDashboard.putData(
                armSubsystem
                        .runDynamicArmSysId(Direction.kReverse)
                        .withName("Arm SysId Dynamic Reverse"));
        SmartDashboard.putData(
                armSubsystem
                        .runQStaticArmSysId(Direction.kForward)
                        .withName("Arm SysId Quasistatic Forward"));
        SmartDashboard.putData(
                armSubsystem
                        .runQStaticArmSysId(Direction.kReverse)
                        .withName("Arm SysId Quasistatic Reverse"));

        SmartDashboard.putData(armSubsystem);

        Trigger LEFT_JOYSTICK_BUMP =
                new Trigger(
                        () ->
                                Math.hypot(
                                                operatorController.getLeftXAxis().get(),
                                                operatorController.getLeftYAxis().get())
                                        > 0.3);

        LEFT_JOYSTICK_BUMP.toggleOnTrue(
                lights.runEnd(
                                (() ->
                                        LightsSubsystem.LEDSegment.MainStrip.setStrobeAnimation(
                                                LightsSubsystem.purple, 1)),
                                () -> {})
                        .withTimeout(5)); // L2 tation Lights

        Trigger RIGHT_JOYSTICK_BUMP =
                new Trigger(
                        () ->
                                Math.hypot(
                                                operatorController.getRightXAxis().get(),
                                                operatorController.getRightYAxis().get())
                                        > 0.3);

        RIGHT_JOYSTICK_BUMP.toggleOnTrue(
                lights.runEnd(
                        (() -> LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(1)),
                        () -> {})); // L3 Rainbow

        operatorController.getStart().onTrue(modeManager.goTo(Position.Climb));

        operatorController.getDPadDown().onTrue(modeManager.goTo(Position.Home));

        operatorController.getStart().onTrue(modeManager.goTo(Position.Climb));
        operatorController.getLeftTrigger().onTrue(modeManager.goTo(Position.Algae3));
        operatorController.getRightTrigger().onTrue(modeManager.goTo(Position.Algae2));

        // operatorController
        //         .getDPadUp()
        //         .onTrue(
        //
        // modeManager.goTo(Position.Handoff).alongWith(gripperSubsystem.intakeUntilPiece()));

        // operatorController.getDPadUp().whileTrue(getAutonomousCommand())

        Trigger DPadUp = operatorController.getDPadUp();
        Trigger Handoffing = new Trigger(gripperSubsystem::intaking);

        DPadUp.onTrue(modeManager.goTo(Position.Handoff));

        DPadUp.onTrue(gripperSubsystem.intakeUntilPiece());
        Handoffing.whileTrue(
                Commands.run(
                        () ->
                                LightsSubsystem.LEDSegment.MainStrip.setStrobeAnimation(
                                        LightsSubsystem.blue, 1),
                        lights));

        // operatorController.getDPadRight().onTrue(modeManager.goTo(Position.Algae2));

        operatorController.getY().onTrue(modeManager.goTo(Position.L4));
        operatorController.getX().onTrue(modeManager.goTo(Position.L3));
        operatorController.getB().onTrue(modeManager.goTo(Position.L2));
        operatorController.getA().onTrue(modeManager.goTo(Position.L1));
        // Driver Align Bindings, for a different/later day
        // CORAL.and(leftDriveController.getTrigger()).whileTrue(alignToReef(9, 0));

        // Climb Bindings
        leftDriveController.getLeftThumb().whileTrue(climberSubsystem.moveClimberDownVoltage());
        leftDriveController.getRightThumb().whileTrue(climberSubsystem.moveClimberUpVoltage());

        operatorController
                .getLeftBumper()
                .onTrue(Commands.runOnce(() -> modeManager.setScoringMode(ScoringMode.LeftCoral)));
        operatorController
                .getBack()
                .onTrue(Commands.runOnce(() -> modeManager.setScoringMode(ScoringMode.Algae)));
        operatorController
                .getRightBumper()
                .onTrue(Commands.runOnce(() -> modeManager.setScoringMode(ScoringMode.RightCoral)));
        // operatorController
        //         .getRightTrigger()
        //         .onTrue([]\
        //                 Commands.runOnce(
        //                         () -> modeManager.setScoringMode(ScoringMode.Algae),
        // modeManager));

        lights.setAlgaeModeSupplier(gripperSubsystem.HAS_PIECE);

        rightDriveController
                .getBottomThumb()
                .and(() -> modeManager.getCurrentScoringMode() == ScoringMode.LeftCoral)
                .whileTrue(alignToReef(AligningConstants.leftOffset));
        rightDriveController
                .getBottomThumb()
                .and(() -> modeManager.getCurrentScoringMode() == ScoringMode.RightCoral)
                .whileTrue(alignToReef(AligningConstants.rightOffset));
        rightDriveController
                 .getBottomThumb()
                 .and(() -> modeManager.getCurrentScoringMode() == ScoringMode.Algae)
                 .whileTrue(alignToReef(AligningConstants.centerOffset));

        // leftDriveController.getLeftBottomMiddle().onTrue(climberSubsystem.zeroClimberCommand());
        rightDriveController.getLeftBottomMiddle().onTrue(modeManager.goTo(Position.Start));
        leftDriveController.getLeftTopMiddle().whileTrue(climberSubsystem.climberTuneable());

        rightDriveController
                .getTrigger()
                .and(() -> modeManager.targetPosition != Position.L1 && modeManager.targetPosition != Position.Algae2 && modeManager.targetPosition != Position.Algae3)
                .whileTrue(gripperSubsystem.intake(GripperConstants.placeVoltage));

        rightDriveController
                .getLeftThumb()
                .and(() -> modeManager.targetPosition != Position.L1)
                .whileTrue(gripperSubsystem.ejectReverse(GripperConstants.placeVoltage));

        operatorController.getDPadLeft().whileTrue(gripperSubsystem.intake(1.5));

        leftDriveController.getLeftBottomMiddle().whileTrue(elevatorSubsystem.setVoltage(1.5));
        leftDriveController.getLeftBottomRight().whileTrue(elevatorSubsystem.setVoltage(-1.5));

        operatorController.getDPadRight().whileTrue(gripperSubsystem.intake(-1.5));

        rightDriveController
                .getTrigger()
                .and(() -> modeManager.targetPosition == Position.L1)
                .whileTrue(gripperSubsystem.setVoltage(1, 4));
//voltage for algae clear
        rightDriveController
                .getTrigger()
                .and(() -> modeManager.targetPosition == Position.Algae2)
                .whileTrue(gripperSubsystem.setVoltage(-6, -6));

        rightDriveController
                .getTrigger()
                .and(() -> modeManager.targetPosition == Position.Algae3)
                .whileTrue(gripperSubsystem.setVoltage(-6, -6));
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
        // Commands.none();
    }

    public Command alignToReef(int tag, double offset, Rotation2d rotOffset) {
        Pose2d alignmentPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(tag)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(AligningConstants.reefDistance, offset),
                                        rotOffset));
        return new AlignToReef(
                drivetrain,
                leftJoystickVelocityX,
                leftJoystickVelocityY,
                0,
                alignmentPose,
                Rotation2d.kPi);
    }

    public Command alignToReef(int tag, double offset) {
        return alignToReef(tag, offset, Rotation2d.kZero);
    }

    // Automatically chooses closest tag
    public Command alignToReef(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = drivetrain.findNearestAprilTagPose();
                    return new AlignToReef(
                            drivetrain,
                            leftJoystickVelocityX,
                            leftJoystickVelocityY,
                            offset,
                            alignmentPose,
                            Rotation2d.kPi);
                },
                Set.of(drivetrain));
    }

    public Command alignAndDriveToReef(int tag, double offset) {
        Pose2d alignmentPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(tag)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(AligningConstants.reefDistance, offset),
                                        new Rotation2d()));
        return new AlignAndDriveToReef(drivetrain, 0, alignmentPose, Rotation2d.kPi);
    }
}
