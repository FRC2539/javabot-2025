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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
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
import frc.robot.subsystems.vision.VisionIOLimelight;
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

    private Trigger DROP_TRIGGER;
    private Trigger AUTO_ALIGNED;
    private Trigger USING_AUTO_ALIGN;
    private Trigger AUTO_DRIVER_TRIGGER;

    private InternalButton normalRelease = new InternalButton();

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

        DROP_TRIGGER = leftDriveController.getTrigger();

        final Trigger EJECT_TRIGGER = rightDriveController.getTrigger();
        final Trigger ALIGN_TRIGGER = rightDriveController.getBottomThumb();
        final Trigger NORMAL_ALIGN =
                (stateManager
                                .STATION_GRIPPER
                                .or(stateManager.STATION_HANDOFF)
                                .or(stateManager.PROCESSOR))
                        .negate();

        USING_AUTO_ALIGN = new Trigger(ALIGN_TRIGGER);

        AUTO_ALIGNED =
                new Trigger(
                        () -> {
                            return AligningConstants.robotInPlace(
                                    drivetrain.getRobotPose(),
                                    stateManager.getLastScoringPose(),
                                    stateManager.getLastScoringOffset());
                        });

        AUTO_DRIVER_TRIGGER = (USING_AUTO_ALIGN.negate().or(AUTO_ALIGNED)).and(DROP_TRIGGER);

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

        // Coral Mode Bindings
        final Trigger CORAL = stateManager.LEFT_CORAL.or(stateManager.RIGHT_CORAL);
        final Trigger ALGAE = stateManager.ALGAE;
        final Trigger ARMWRIST = stateManager.ARMWRIST;
        ARMWRIST.and(operatorController.getY()).whileTrue(armSubsystem.armpivotUp());
        ARMWRIST.and(operatorController.getA()).whileTrue(armSubsystem.armpivotDown());
        ARMWRIST.and(operatorController.getX()).whileTrue(wristSubsystem.turnWristLeft());
        ARMWRIST.and(operatorController.getB()).whileTrue(wristSubsystem.turnWristRight());

        // CORAL.and(operatorController.getY())
        //         .onTrue(stateManager.moveToPosition(Position.L4Prep))
        //         .onFalse(stateManager.moveToPosition(Position.L4));
        // CORAL.and(operatorController.getX())
        //         .onTrue(stateManager.moveToPosition(Position.L3Prep))
        //         .onFalse(stateManager.moveToPosition(Position.L3));
        // CORAL.and(operatorController.getB())
        //         .onTrue(stateManager.moveToPosition(Position.L2Prep))
        //         .onFalse(stateManager.moveToPosition(Position.L2));
        // CORAL.and(operatorController.getA()).onTrue(stateManager.moveToPosition(Position.L1));

        operatorController.getLeftTrigger().onTrue(stateManager.moveToPosition(Position.Climb));

        bindPlaceSeq(CORAL.and(operatorController.getY()), Position.L4Prep, Position.L4, 0.1);

        bindPlaceSeq(CORAL.and(operatorController.getX()), Position.L3Prep, Position.L3, 0.1);

        bindPlaceSeq(CORAL.and(operatorController.getB()), Position.L2Prep, Position.L2, 0.1);

        operatorController.getA().onTrue(stateManager.moveToPosition(Position.L1));

        CORAL.and(operatorController.getStart())
                .onTrue(stateManager.moveToPosition(Position.Source));

        CORAL.and(operatorController.getDPadDown())
                .onTrue(stateManager.moveToPosition(Position.Home));
        CORAL.and(operatorController.getDPadUp())
                .onFalse(stateManager.moveToPosition(Position.HandoffPrep))
                .onTrue(stateManager.moveToPosition(Position.Handoff));
        CORAL.and(operatorController.getDPadUp()).whileTrue(gripperSubsystem.intakeSpinCoral());

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
        // ALGAE.and(operatorController.getDPadUp())
        //         .onTrue(stateManager.moveToPosition(Position.Handoff));
        ALGAE.and(operatorController.getDPadLeft())
                .onTrue(stateManager.moveToPosition(Position.Quick34));
        ALGAE.and(operatorController.getDPadRight())
                .onTrue(stateManager.moveToPosition(Position.Quick23));

        operatorController
                .getBack()
                .whileTrue(
                        stateManager
                                .moveToPosition(Position.HandoffSus)
                                .alongWith(gripperSubsystem.intakeSpinCoral()));

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

        lights.setAlgaeModeSupplier(gripperSubsystem.HAS_PIECE);

        CORAL.and(rightDriveController.getLeftThumb())
                .whileTrue(
                        gripperSubsystem
                                .intakeSpinCoral()
                                .withDeadline(
                                        Commands.waitSeconds(0.2)
                                                .andThen(
                                                        Commands.waitUntil(
                                                                gripperSubsystem.HAS_PIECE))));
        CORAL.and(EJECT_TRIGGER).whileTrue(gripperSubsystem.ejectSpinCoral());

        ALGAE.and(rightDriveController.getLeftThumb()).onTrue(gripperSubsystem.intakeSpinAlgae());
        ALGAE.and(EJECT_TRIGGER)
                .and(stateManager.PROCESSOR)
                .whileTrue(gripperSubsystem.slowEjectSpinAlgae());
        ALGAE.and(EJECT_TRIGGER)
                .and(stateManager.PROCESSOR.negate())
                .whileTrue(gripperSubsystem.ejectSpinAlgae());

        ALIGN_TRIGGER.onTrue(
                Commands.runOnce(
                        () ->
                                stateManager.setLastScoringPose(
                                        drivetrain.findNearestAprilTagPose())));

        stateManager
                .LEFT_CORAL
                .and(ALIGN_TRIGGER)
                .whileTrue(alignToReef(AligningConstants.leftOffset));

        stateManager
                .ALGAE
                .and(ALIGN_TRIGGER)
                .whileTrue(alignToReef(AligningConstants.centerOffset));

        stateManager
                .RIGHT_CORAL
                .and(ALIGN_TRIGGER)
                .whileTrue(alignToReef(AligningConstants.rightOffset));

        // stateManager
        //         .ALGAE
        //         .and(ALIGN_TRIGGER.and(stateManager.PROCESSOR))
        //         .whileTrue(alignToProcessor());

        // stateManager
        //         .LEFT_CORAL
        //         .and(ALIGN_TRIGGER.and(stateManager.STATION_GRIPPER))
        //         .whileTrue(alignToStation(0.6, Rotation2d.kZero));

        // stateManager
        //         .ALGAE
        //         .and(ALIGN_TRIGGER.and(stateManager.STATION_GRIPPER))
        //         .whileTrue(alignToStation(0.0, Rotation2d.kZero));

        // stateManager
        //         .RIGHT_CORAL
        //         .and(ALIGN_TRIGGER.and(stateManager.STATION_GRIPPER))
        //         .whileTrue(alignToStation(-0.6, Rotation2d.kZero));

        // stateManager
        //         .LEFT_CORAL
        //         .and(ALIGN_TRIGGER.and(stateManager.STATION_HANDOFF))
        //         .whileTrue(alignToStation(0.6, Rotation2d.k180deg));

        // stateManager
        //         .ALGAE
        //         .and(ALIGN_TRIGGER.and(stateManager.STATION_HANDOFF))
        //         .whileTrue(alignToStation(0.0, Rotation2d.k180deg));

        // stateManager
        //         .RIGHT_CORAL
        //         .and(ALIGN_TRIGGER.and(stateManager.STATION_HANDOFF))
        //         .whileTrue(alignToStation(-0.6, Rotation2d.k180deg));

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

    private void bindPlaceSeq(Trigger button, Position prep, Position end, double timeout) {
        (button)
                .onTrue(
                        (stateManager
                                        .moveToPosition(prep)
                                        .until(DROP_TRIGGER)
                                        .andThen(
                                                stateManager
                                                        .moveToPosition(end)
                                                        //                         .alongWith(
                                                        //
                                                        // Commands.waitSeconds(timeout)
                                                        //
                                                        // .andThen(
                                                        //
                                                        //       gripperSubsystem
                                                        //
                                                        //               .ejectSpinCoral()))
                                                        .until(DROP_TRIGGER.negate())))
                                .repeatedly()
                                .beforeStarting(() -> normalRelease.setPressed(false))
                                .finallyDo(() -> normalRelease.setPressed(true)));
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
                            Pose2d alignmentPose =
                                    stateManager
                                            .getLastScoringPose()
                                            .plus(
                                                    new Transform2d(
                                                            new Translation2d(
                                                                    AligningConstants.reefDistance,
                                                                    offset),
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
                        Set.of(drivetrain))
                .beforeStarting(() -> stateManager.setLastScoringOffset(offset));
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

    public Command alignToProcessor() {
        // getAlignmentcolor 3 = r
        // set a trigger based on the thumbpad
        //
        return Commands.either(
                alignToReef(3, 0),
                alignToReef(16, 0),
                () -> {
                    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                });
    }

    public Command alignToStation(double offset, Rotation2d rotOffset) {
        // getAlignmentcolor 3 = r
        // set a trigger based on the thumbpad
        //
        return Commands.defer(
                () -> {
                    int closestTag = 0;
                    Pose2d currentPose = drivetrain.getRobotPose();
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                        if (currentPose.getY() > 4.026) {
                            closestTag = 2;
                        } else {
                            closestTag = 1;
                        }
                    } else {
                        if (currentPose.getY() > 4.026) {
                            closestTag = 13;
                        } else {
                            closestTag = 12;
                        }
                    }
                    return alignToReef(closestTag, offset, rotOffset);
                },
                Set.of(drivetrain));
    }
}
