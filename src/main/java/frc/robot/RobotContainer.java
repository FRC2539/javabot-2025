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
import frc.robot.subsystems.ModeManager.ModeManager;
import frc.robot.subsystems.ModeManager.ModeManager.CoralAlgaeMode;
import frc.robot.subsystems.ModeManager.ModeManager.State.Position;
import frc.robot.subsystems.arm.ArmPivotIOSim;
import frc.robot.subsystems.arm.ArmPivotIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberHeadIONeo550;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.GripperIONeo550;
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

    //public Auto auto; // #146: Pass in RobotContainer
    public IntakeSubsystem intakeSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public ClimberSubsystem climberSubsystem;
    public ArmSubsystem armSubsystem;

    public Vision vision;
    public LightsSubsystem lights;

    public ModeManager stateManager;
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
                            //     new DummyPhotonCamera(),
                            //     new DummyPhotonCamera(),
                            //     new DummyPhotonCamera());
                            new VisionIOLimelight(
                                    VisionConstants.camera0Name,
                                    () -> drivetrain.getRobotPose().getRotation()),
                            new VisionIOLimelight(
                                    VisionConstants.camera1Name,
                                    () -> drivetrain.getRobotPose().getRotation()),
                            new DummyPhotonCamera());
            elevatorSubsystem =
                    new ElevatorSubsystem(new ElevatorIOTalonFX()); // new ElevatorIOTalonFX());
            armSubsystem = new ArmSubsystem(new ArmPivotIOTalonFX());

            gripperSubsystem = new GripperSubsystem(new GripperIONeo550());

            climberSubsystem =
                    new ClimberSubsystem(
                            new ClimberIOTalonFX(),
                            new ClimberHeadIONeo550()); // new ClimberIOTalonFX(), new
            // ClimberHeadIONeo550());
            lights = new LightsSubsystem();

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
            intakeSubsystem = new IntakeSubsystem(new IntakeRollerIOSim(), new FlipperIOSim());
            climberSubsystem =
                    new ClimberSubsystem(new ClimberIOTalonFX(), new ClimberHeadIONeo550());
            lights = new LightsSubsystem();
        }

        // auto = new Auto(drivetrain, this);

        stateManager = new ModeManager(elevatorSubsystem, armSubsystem);
        stateManager.setDefaultCommand(Commands.runOnce(() -> {}, stateManager));
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
        //final Trigger ALIGN_TRIGGER = rightDriveController.getBottomThumb();
        // final Trigger NORMAL_ALIGN =
        //         (stateManager.STATION_GRIPPER.or(stateManager.PROCESSOR)).negate();

        //USING_AUTO_ALIGN = new Trigger(ALIGN_TRIGGER);

        

        //AUTO_DRIVER_TRIGGER = (USING_AUTO_ALIGN.negate().or(AUTO_ALIGNED)).and(DROP_TRIGGER);

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
        operatorController.getLeftBumper().onTrue(Commands.runOnce(() -> stateManager.setMode(CoralAlgaeMode.LeftCoral), stateManager));
        operatorController.getRightBumper().onTrue(Commands.runOnce(() -> stateManager.setMode(CoralAlgaeMode.RightCoral), stateManager));
        operatorController.getRightTrigger().onTrue(Commands.runOnce(() -> stateManager.setMode(CoralAlgaeMode.Algae), stateManager));

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

        // CORAL.and(operatorController.getY())
        //         .onTrue(stateManager.moveToPosition(Position.L4Prep))
        //         .onFalse(stateManager.moveToPosition(Position.L4));
        // CORAL.and(operatorController.getX())
        //         .onTrue(stateManager.moveToPosition(Position.L3Prep))
        //         .onFalse(stateManager.moveToPosition(Position.L3));
        // CORAL

        operatorController.getLeftTrigger().onTrue(stateManager.setGoal(Position.Climb));

        operatorController.getY().onTrue(stateManager.setGoal(Position.L4));

        operatorController.getY().onTrue(stateManager.setG
        oal(Position.L4));

        //operatorController.getY().onTrue(Commands.runOnce(() -> System.out.println("Y PRESSED")));
        operatorController.getX().onTrue(stateManager.setGoal(Position.L3));

        operatorController.getB().onTrue(stateManager.setGoal(Position.L2));

        operatorController.getA().onTrue(stateManager.setGoal(Position.L1));

        operatorController.getDPadDown().onTrue(stateManager.setGoal(Position.Home));
        operatorController.getDPadUp().onTrue(stateManager.setGoal(Position.Handoff));

        operatorController.getDPadLeft().onTrue(stateManager.setGoal(Position.Quick3));
        operatorController.getDPadRight().onTrue(stateManager.setGoal(Position.Quick2));

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

        lights.setAlgaeModeSupplier(gripperSubsystem.HAS_PIECE);

        (rightDriveController.getLeftThumb())
                .whileTrue(
                        gripperSubsystem
                                .ejectReverse(12)
                                .withDeadline(
                                        Commands.waitSeconds(0.2)
                                                .andThen(
                                                        Commands.waitUntil(
                                                                gripperSubsystem.HAS_PIECE))));
        (EJECT_TRIGGER).whileTrue(gripperSubsystem.ejectReverse(12));

        rightDriveController.getBottomThumb().whileTrue(alignToReef(stateManager.getAligningOffset()));

        // Technical Bindings

        leftDriveController.getLeftBottomMiddle().onTrue(climberSubsystem.zeroClimberCommand());
        rightDriveController
                .getLeftBottomMiddle()
                .onTrue(stateManager.setGoal(Position.Start));
        leftDriveController.getLeftTopMiddle().whileTrue(climberSubsystem.climberTuneable());

        rightDriveController
                .getLeftTopLeft()
                .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

        // leftDriveController.getLeftBottomLeft().whileTrue(wristSubsystem.tunablePose());
        // leftDriveController.getLeftTopRight().whileTrue(wristSubsystem.tuneableVoltage());

        // leftDriveController.getLeftBottomLeft().whileTrue(intakeSubsystem.rollerTuneable());
        // leftDriveController.getLeftTopRight().whileTrue(intakeSubsystem.flipperTuneable());

        leftDriveController.getLeftBottomRight().onTrue(intakeSubsystem.zeroflipper());

        //leftDriveController.getLeftTopLeft().whileTrue(gripperSubsystem.gripperTuneable());
        
            

        SmartDashboard.putData(stateManager);
        

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

//     public Command getAutonomousCommand() {
//         return auto.getAuto();
//     }

    public Command alignToReef(int tag, double offset, Rotation2d rotOffset) {
        Pose2d alignmentPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(tag)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(Units.feetToMeters(3) / 2, offset),
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
                                    VisionConstants.aprilTagLayout.getTagPose(3).get().toPose2d(); 
                                
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

}
