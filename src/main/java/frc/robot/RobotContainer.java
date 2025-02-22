// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.vision.PinholeModel3D;
import frc.robot.commands.AlignAndDriveToReef;
import frc.robot.commands.AlignToPiece;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.constants.AligningConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.ControllerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager;
import frc.robot.subsystems.ModeManager.SuperstructureStateManager.SuperstructureState.Position;
import frc.robot.subsystems.arm.ArmPivotIOSim;
import frc.robot.subsystems.arm.ArmPivotIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.chute.ChuteIONeo550;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.chute.ChuteSubsystem;
import frc.robot.subsystems.climber.ClimberHeadIONeo550;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperIOFalcon;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.intake.FlipperIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.DummyPhotonCamera;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSimML;
import frc.robot.subsystems.wrist.WristIONeo550;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.Elastic;
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
    public IntakeSubsystem intakeSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public ClimberSubsystem climberSubsystem;
    public ArmSubsystem armSubsystem;
    public WristSubsystem wristSubsystem;
    public Vision vision;
    public LightsSubsystem lights;
    public ChuteSubsystem chuteSubsystem;
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
                            new DummyPhotonCamera(),
                            new DummyPhotonCamera(),
                            new DummyPhotonCamera());
            //     new VisionIOLimelight(
            //             VisionConstants.camera0Name,
            //             () -> drivetrain.getRobotPose().getRotation()),
            //     new VisionIOLimelight(
            //             VisionConstants.camera1Name,
            //             () -> drivetrain.getRobotPose().getRotation()),
            //     new VisionIOLimelight(
            //             VisionConstants.camera2Name,
            //             () -> drivetrain.getRobotPose().getRotation()));
            gripperSubsystem =
                    new GripperSubsystem(new GripperIOFalcon()); // new GripperIOFalcon());
            elevatorSubsystem =
                    new ElevatorSubsystem(new ElevatorIOTalonFX()); // new ElevatorIOTalonFX());
            armSubsystem = new ArmSubsystem(new ArmPivotIOTalonFX());
            wristSubsystem = new WristSubsystem(new WristIONeo550()); // new WristIONeo550());
            climberSubsystem =
                    new ClimberSubsystem(
                            new ClimberIOTalonFX(),
                            new ClimberHeadIONeo550()); // new ClimberIOTalonFX(), new
            // ClimberHeadIONeo550());
            lights = new LightsSubsystem();
            chuteSubsystem = new ChuteSubsystem(new ChuteIONeo550()); // new ChuteIONeo550());

            intakeSubsystem = new IntakeSubsystem(new IntakeRollerIOSim(), new FlipperIOSim());
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
            wristSubsystem = new WristSubsystem(new WristIOSim());
            intakeSubsystem = new IntakeSubsystem(new IntakeRollerIOSim(), new FlipperIOSim());
            climberSubsystem =
                    new ClimberSubsystem(new ClimberIOTalonFX(), new ClimberHeadIONeo550());
            lights = new LightsSubsystem();
            chuteSubsystem = new ChuteSubsystem(new ChuteIOSim());
        }

        stateManager =
                new SuperstructureStateManager(
                        elevatorSubsystem, armSubsystem, wristSubsystem, chuteSubsystem);

        auto = new Auto(drivetrain, this);

        wristSubsystem.elevatorHeight = () -> elevatorSubsystem.getPosition();
        wristSubsystem.armHeight = () -> armSubsystem.getPosition();

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

        // drive.withVelocityX(-leftDriveController.getYAxis().get() *
        // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive forward with negative Y
        // (forward)
        // .withVelocityY(-leftDriveController.getXAxis().get() *
        // GlobalConstants.MAX_TRANSLATIONAL_SPEED) // Drive left with negative X (left)
        // .withRotationalRate(-rightDriveController.getXAxis().get() *
        // GlobalConstants.MAX_ROTATIONAL_SPEED) // Drive counterclockwise with negative
        // X (left)

        // operatorController.getA().whileTrue(drivetrain.applyRequest(() -> brake));
        // operatorController.getA().onTrue(new alignToTargetX(drivetrain, vision, 10,
        // 0));

        // operatorController
        // .getA()
        // .onTrue(
        // new AlignToAngle(
        // drivetrain,
        // new Rotation2d(),
        // true,
        // leftJoystickVelocityX,
        // leftJoystickVelocityY)
        // .andThen(
        // new alignToTargetX(
        // drivetrain, vision, 10, 0,
        // leftJoystickVelocityX)));

        // operatorController.getA().toggleOnTrue(alignToReef(9, 0));
        // leftDriveController.getBottomThumb().whileTrue(alignToReef(9, 0));
        // leftDriveController.getRightThumb().whileTrue(alignToReef(9, 0.4));
        // leftDriveController.getLeftThumb().whileTrue(alignToReef(9, -0.4));
        // leftDriveController.getBottomThumb().whileTrue(alignAndDriveToReef(19, 0));
        // operatorController
        // .getB()
        // .whileTrue(
        // drivetrain.applyRequest(
        // () ->
        // point.withModuleDirection(
        // new Rotation2d(
        // -operatorController.getLeftYAxis().get(),
        // -operatorController
        // .getLeftXAxis()
        // .get()))));

        // leftDriveController
        // .getTrigger()
        // .whileTrue(
        // new WheelRadiusCharacterization(
        // WheelRadiusCharacterization.Direction.CLOCKWISE, drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // operatorController
        // .getBack()
        // .and(operatorController.getY())
        // .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // operatorController
        // .getBack()
        // .and(operatorController.getX())
        // .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // operatorController
        // .getStart()
        // .and(operatorController.getY())
        // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // operatorController
        // .getStart()
        // .and(operatorController.getX())
        // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
        // operatorController
        // operatorController.getA().onTrue(stateManager.moveToPosition(Position.L4));
        // operatorController.getB().onTrue(stateManager.moveToPosition(Position.L3));
        // operatorController.getX().onTrue(stateManager.moveToPosition(Position.Source));
        // operatorController.getY().onTrue(stateManager.moveToPosition(Position.Home));

        // reset the field-centric heading on left bumper press
        // operatorController
        // .getLeftBumper()
        // .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // operatorController
        // .getRightBumper()
        // .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

        // Operator Mode Setting
        operatorController.getLeftBumper().onTrue(stateManager.setLeftCoralMode());
        operatorController.getRightBumper().onTrue(stateManager.setRightCoralMode());
        operatorController.getRightTrigger().onTrue(stateManager.setAlgaeMode());
        leftDriveController.getRightTopRight().onTrue(stateManager.setArmWristMode());
        operatorController
                .getLeftJoystick()
                .toggleOnTrue(
                        Commands.runOnce(
                                (() ->
                                        LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(
                                                1)))); // L3 Rainbow
        operatorController
                .getLeftTrigger()
                .whileTrue(
                        Commands.runOnce(
                                (() ->
                                        LightsSubsystem.LEDSegment.MainStrip.setStrobeAnimation(
                                                LightsSubsystem.purple, 1)))); // L2 tation Lights

        // Coral Mode Bindings
        final Trigger CORAL = stateManager.LEFT_CORAL.or(stateManager.RIGHT_CORAL);
        final Trigger ALGAE = stateManager.ALGAE;
        final Trigger ARMWRIST = stateManager.ARMWRIST;
        ARMWRIST.and(operatorController.getY()).whileTrue(armSubsystem.armpivotUp());
        ARMWRIST.and(operatorController.getA()).whileTrue(armSubsystem.armpivotDown());
        ARMWRIST.and(operatorController.getX()).whileTrue(wristSubsystem.turnWristLeft());
        ARMWRIST.and(operatorController.getB()).whileTrue(wristSubsystem.turnWristRight());

        CORAL.and(operatorController.getY())
                .onTrue(stateManager.moveToPosition(Position.L4Prep))
                .onFalse(stateManager.moveToPosition(Position.L4));
        CORAL.and(operatorController.getX())
                .onTrue(stateManager.moveToPosition(Position.L3Prep))
                .onFalse(stateManager.moveToPosition(Position.L3));
        CORAL.and(operatorController.getB())
                .onTrue(stateManager.moveToPosition(Position.L2Prep))
                .onFalse(stateManager.moveToPosition(Position.L2));
        CORAL.and(operatorController.getA()).onTrue(stateManager.moveToPosition(Position.L1));
        CORAL.and(operatorController.getStart())
                .onTrue(stateManager.moveToPosition(Position.Source));

        CORAL.and(operatorController.getDPadDown())
                .onTrue(stateManager.moveToPosition(Position.Home));
        CORAL.and(operatorController.getDPadUp())
                .onTrue(stateManager.moveToPosition(Position.HandoffPrep))
                .onFalse(stateManager.moveToPosition(Position.Handoff));
        CORAL.and(operatorController.getDPadLeft()).onTrue(chuteSubsystem.moveChuteUp());
        CORAL.and(operatorController.getDPadRight()).onTrue(chuteSubsystem.moveChuteDown());

        // ALGAE.and(operatorController.getY()).onTrue(stateManager.moveToPosition(Position.NetAlgae));
        ALGAE.and(operatorController.getX()).onTrue(stateManager.moveToPosition(Position.L3Algae));
        ALGAE.and(operatorController.getB()).onTrue(stateManager.moveToPosition(Position.L2Algae));
        ALGAE.and(operatorController.getA())
                .onTrue(stateManager.moveToPosition(Position.Processor));
        ALGAE.and(operatorController.getStart())
                .onTrue(stateManager.moveToPosition(Position.GroundAlgae));
        ALGAE.and(operatorController.getDPadDown())
                .onTrue(stateManager.moveToPosition(Position.AlgaeHome));
        ALGAE.and(operatorController.getDPadUp())
                .onTrue(stateManager.moveToPosition(Position.Handoff));
        ALGAE.and(operatorController.getDPadLeft())
                .onTrue(stateManager.moveToPosition(Position.Quick34));
        ALGAE.and(operatorController.getDPadRight())
                .onTrue(stateManager.moveToPosition(Position.Quick23));

        // operatorController.getBack().onTrue(wristSubsystem.flipWristPosition());

        // Driver Align Bindings, for a different/later day
        // CORAL.and(leftDriveController.getTrigger()).whileTrue(alignToReef(9, 0));

        // Climb Bindings
        leftDriveController.getLeftThumb().whileTrue(climberSubsystem.moveClimberDownVoltage());
        leftDriveController.getRightThumb().whileTrue(climberSubsystem.moveClimberUpVoltage());
        leftDriveController.getBottomThumb().whileTrue(climberSubsystem.intakeCage());

        // leftDriveController.getBottomThumb().whileTrue(alignToPiece());

        // Intake Bindings
        // rightDriveController
        //         .getLeftThumb()
        //         .whileTrue(intakeSubsystem.openAndRun().alongWith(alignToPiece()));
        // rightDriveController.getRightThumb().whileTrue(intakeSubsystem.openAndEject());

        CORAL.and(rightDriveController.getBottomThumb())
                .whileTrue(
                        gripperSubsystem
                                .intakeSpinCoral()
                                .withDeadline(
                                        Commands.waitSeconds(0.2)
                                                .andThen(
                                                        Commands.waitUntil(
                                                                gripperSubsystem.HAS_PIECE))));
        CORAL.and(rightDriveController.getTrigger()).whileTrue(gripperSubsystem.ejectSpinCoral());

        ALGAE.and(rightDriveController.getBottomThumb()).onTrue(gripperSubsystem.intakeSpinAlgae());
        ALGAE.and(rightDriveController.getTrigger())
                .and(stateManager.PROCESSOR)
                .whileTrue(gripperSubsystem.slowEjectSpinAlgae());
        ALGAE.and(rightDriveController.getTrigger())
                .and(stateManager.PROCESSOR.negate())
                .whileTrue(gripperSubsystem.ejectSpinAlgae());

        leftDriveController
                .getTrigger()
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        stateManager.setLastScoringPose(
                                                drivetrain.findNearestAprilTagPose())));

        stateManager
                .LEFT_CORAL
                .and(leftDriveController.getTrigger())
                .whileTrue(alignToReef(AligningConstants.leftOffset));

        stateManager
                .ALGAE
                .and(leftDriveController.getTrigger())
                .whileTrue(alignToReef(AligningConstants.centerOffset));

        stateManager
                .RIGHT_CORAL
                .and(leftDriveController.getTrigger())
                .whileTrue(alignToReef(AligningConstants.rightOffset));

        // Technical Bindings

        leftDriveController.getLeftBottomMiddle().onTrue(climberSubsystem.zeroClimberCommand());
        rightDriveController
                .getLeftBottomMiddle()
                .onTrue(stateManager.moveToPosition(Position.Start));
        leftDriveController.getLeftTopMiddle().whileTrue(climberSubsystem.climberTuneable());

        rightDriveController
                .getLeftTopLeft()
                .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

        // leftDriveController.getLeftBottomLeft().whileTrue(wristSubsystem.tunablePose());
        // leftDriveController.getLeftTopRight().whileTrue(wristSubsystem.tuneableVoltage());

        // leftDriveController.getLeftBottomLeft().whileTrue(intakeSubsystem.rollerTuneable());
        // leftDriveController.getLeftTopRight().whileTrue(intakeSubsystem.flipperTuneable());

        leftDriveController.getLeftBottomLeft().whileTrue(chuteSubsystem.moveChuteUp());
        leftDriveController.getLeftTopRight().whileTrue(chuteSubsystem.moveChuteDown());

        leftDriveController.getLeftBottomRight().onTrue(intakeSubsystem.zeroflipper());

        leftDriveController.getLeftTopLeft().whileTrue(gripperSubsystem.gripperTuneable());
        {
            var tunableCommand =
                    Commands.runOnce(
                                    () -> {
                                        Elastic.sendNotification(
                                                new Elastic.Notification(
                                                        Elastic.Notification.NotificationLevel.INFO,
                                                        "Scheduled Supestructure Tunable",
                                                        "YAYYAYYA."));
                                    })
                            .andThen(stateManager.moveToTunablePosition());

            tunableCommand.setName("Tunable Superstructure");

            leftDriveController
                    .getRightTopLeft()
                    .onTrue(
                            Commands.runOnce(
                                    () -> {
                                        tunableCommand.cancel();
                                        tunableCommand.schedule();
                                    }));

            SmartDashboard.putData(tunableCommand);

            SmartDashboard.putData(stateManager);
        }

        // leftDriveController.getRightBottomLeft().onTrue(elevatorSubsystem.zeroElevatorCommand());
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
        Pose2d alignmentPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(tag)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(Units.feetToMeters(3) / 2, offset),
                                        new Rotation2d()));
        return new AlignToReef(
                drivetrain,
                leftJoystickVelocityX,
                leftJoystickVelocityY,
                0,
                alignmentPose,
                Rotation2d.kPi); // Skibidi
    }

    // Automatically chooses closest tag
    public Command alignToReef(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose =
                            stateManager
                                    .getLastScoringPose()
                                    .plus(
                                            new Transform2d(
                                                    new Translation2d(
                                                            Units.feetToMeters(3) / 2, offset),
                                                    new Rotation2d()));
                    //         return new AlignAndDriveToReef(drivetrain, 0, alignmentPose,
                    // Rotation2d.kPi);
                    return new AlignToReef(
                            drivetrain,
                            leftJoystickVelocityX,
                            leftJoystickVelocityY,
                            0,
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
                                        new Translation2d(Units.feetToMeters(3) / 2, offset),
                                        new Rotation2d()));
        return new AlignAndDriveToReef(drivetrain, 0, alignmentPose, Rotation2d.kPi);
    }

    public Command alignToPiece() {
        Supplier<Pose2d> piecePositionSupplier =
                () -> {
                    var lastObservation = vision.getLastTargetObersevation(2);
                    Pose2d robotPose = drivetrain.getRobotPose();
                    Translation2d lastPieceTranslation =
                            PinholeModel3D.getTranslationToTarget(
                                    new Translation3d(
                                            1,
                                            lastObservation.tx().unaryMinus().getTan(),
                                            lastObservation.ty().getTan()),
                                    VisionConstants.robotToCamera2,
                                    0);
                    Pose2d poseAtTime = robotPose;

                    Pose2d newPiecePose =
                            poseAtTime.plus(
                                    new Transform2d(lastPieceTranslation, new Rotation2d()));

                    return newPiecePose;
                };
        return new AlignToPiece(
                drivetrain,
                driverVelocitySupplier,
                .15,
                piecePositionSupplier,
                Rotation2d.kCCW_90deg);
    }
}
