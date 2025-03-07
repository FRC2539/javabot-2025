package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.constants.VisionConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain implements Subsystem {
    private final TunerSwerveDrivetrain m_drivetrain;

    private final CustomOdometry m_odometry_custom;

    private enum SwerveState {
        ROBOT_RELATIVE("robot relative"),
        ROBOT_RELATIVE_FFW("robot relative feedforwards"),
        FIELD_RELATIVE("field relative"),
        FIELD_RELATIVE_FFW("field relative feedforwards"),
        DRIVER_RELATIVE("driver relative"),
        DRIVER_RELATIVE_ORBIT("driver relative orbit-style"); /*,
        ANTI_TIP_LIMITING */

        public final String label;

        private SwerveState(String label) {
            this.label = label;
        }
    }

    private SwerveState m_swerveState;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.ApplyFieldSpeeds m_applyDriverSpeeds =
            new SwerveRequest.ApplyFieldSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.ApplyFieldSpeeds m_applyFieldSpeeds =
            new SwerveRequest.ApplyFieldSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private final FieldOrientedOrbitSwerveRequest m_applyFieldSpeedsOrbit;
    RobotConfig config; // PathPlanner robot configuration

    private Field2d m_field2d = new Field2d();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null, // Use default ramp rate (1 V/s)
                            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                            null, // Use default timeout (10 s)
                            // Log state with SignalLogger class
                            state ->
                                    SignalLogger.writeString(
                                            "SysIdTranslation_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            output -> setControl(m_translationCharacterization.withVolts(output)),
                            null,
                            this));

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null, // Use default ramp rate (1 V/s)
                            Volts.of(7), // Use dynamic voltage of 7 V
                            null, // Use default timeout (10 s)
                            // Log state with SignalLogger class
                            state ->
                                    SignalLogger.writeString("SysIdSteer_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            volts -> setControl(m_steerCharacterization.withVolts(volts)),
                            null,
                            this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            /* This is in radians per second², but SysId only supports "volts per second" */
                            Volts.of(Math.PI / 6).per(Second),
                            /* This is in radians per second, but SysId only supports "volts" */
                            Volts.of(Math.PI),
                            null, // Use default timeout (10 s)
                            // Log state with SignalLogger class
                            state ->
                                    SignalLogger.writeString(
                                            "SysIdRotation_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            output -> {
                                /* output is actually radians per second, but SysId only supports "volts" */
                                setControl(
                                        m_rotationCharacterization.withRotationalRate(
                                                output.in(Volts)));
                                /* also log the requested output for SysId */
                                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                            },
                            null,
                            this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* The Setpoint Generator */
    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public Pose2d getRobotPose() {
        return getState().Pose;
    }

    public Pose2d findNearestAprilTagPose() {
        // TODO: filter out opposing side tags and non-reef tags
        Pose2d currentPose = getRobotPose();
        Pose2d nearestAprilTagPose = null;
        double nearestDistance = Double.MAX_VALUE;

        Pose2d[] aprilTagPoses = new Pose2d[6];

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            for (int i = 0; i < 6; i++) {
                aprilTagPoses[i] =
                        VisionConstants.aprilTagLayout
                                .getTagPose(GlobalConstants.redReefTagIDs[i])
                                .get()
                                .toPose2d();
            }
        } else {
            for (int i = 0; i < 6; i++) {
                aprilTagPoses[i] =
                        VisionConstants.aprilTagLayout
                                .getTagPose(GlobalConstants.blueReefTagIDs[i])
                                .get()
                                .toPose2d();
            }
        }

        for (Pose2d tagPose : aprilTagPoses) {
            double distance = currentPose.getTranslation().getDistance(tagPose.getTranslation());
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestAprilTagPose = tagPose;
            }
        }

        return nearestAprilTagPose;
    }

    public Rotation2d getOperatorForwardDirection() {
        return m_drivetrain.getOperatorForwardDirection();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.getState().Speeds;
    }

    public SwerveDriveState getState() {
        return m_odometry_custom.m_state;
    }

    double lastTimestamp = 0;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        m_drivetrain = new TunerSwerveDrivetrain(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_odometry_custom =
                new CustomOdometry(new CustomInverseKinematics(m_drivetrain.getModuleLocations()));
        m_drivetrain.registerTelemetry(
                (SwerveDriveState state) -> {
                    m_odometry_custom.odometryFunction(state);
                    m_drivetrain.resetPose(m_odometry_custom.m_currentPose);
                });
        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();

        m_swerveState = SwerveState.DRIVER_RELATIVE;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        m_drivetrain =
                new TunerSwerveDrivetrain(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        m_odometry_custom =
                new CustomOdometry(new CustomInverseKinematics(m_drivetrain.getModuleLocations()));
        m_drivetrain.registerTelemetry(
                (SwerveDriveState state) -> {
                    m_odometry_custom.odometryFunction(state);
                    m_drivetrain.resetPose(m_odometry_custom.m_currentPose);
                });

        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();

        m_swerveState = SwerveState.DRIVER_RELATIVE;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *     y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        m_drivetrain =
                new TunerSwerveDrivetrain(
                        drivetrainConstants,
                        odometryUpdateFrequency,
                        odometryStandardDeviation,
                        visionStandardDeviation,
                        modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_odometry_custom =
                new CustomOdometry(new CustomInverseKinematics(m_drivetrain.getModuleLocations()));
        m_drivetrain.registerTelemetry(
                (SwerveDriveState state) -> {
                    m_odometry_custom.odometryFunction(state);
                    m_drivetrain.resetPose(m_odometry_custom.m_currentPose);
                });

        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();

        m_swerveState = SwerveState.DRIVER_RELATIVE;
    }

    private FieldOrientedOrbitSwerveRequest generateSwerveSetpointConfig() {
        RobotConfig config = GlobalConstants.getRobotConfigPathplanner();

        setpointGenerator =
                new SwerveSetpointGenerator(
                        config, Units.rotationsToRadians(10.0) // max rotational speed
                        );

        ChassisSpeeds currentSpeeds = getState().Speeds;
        SwerveModuleState[] currentStates = getState().ModuleStates;
        SwerveSetpoint previousSetpoint =
                new SwerveSetpoint(
                        currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));

        var request =
                new FieldOrientedOrbitSwerveRequest(
                        setpointGenerator, previousSetpoint, getState().Pose.getRotation());
        request.withDriverOrientation(true);
        return request;
    }

    private double currentMaxAcceleration = 5;
    private double futureMaxAcceleration = 5;

    public ChassisSpeeds limitFieldRelativeSpeeds(ChassisSpeeds inputSpeeds) {
        return limitFieldRelativeSpeeds(inputSpeeds, false);
    }

    public ChassisSpeeds limitFieldRelativeSpeeds(
            ChassisSpeeds inputSpeeds, boolean maintainRotationProportion) {
        // var robotState = getState();
        // double max_speed = Math.hypot(inputSpeeds.vxMetersPerSecond,
        // inputSpeeds.vyMetersPerSecond);
        // double limited_x_speed =
        // antiTipSlewer.getMaxAllowedVelocityDirectional(inputSpeeds.vxMetersPerSecond, true);
        // double limited_y_speed =
        // antiTipSlewer.getMaxAllowedVelocityDirectional(inputSpeeds.vyMetersPerSecond, false);

        // double x_ratio = limited_x_speed / inputSpeeds.vxMetersPerSecond;
        // double y_ratio = limited

        double max_speed_to_allowed_ratio =
                antiTipSlewer.getMaxAllowedVelocityRatio(
                        inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond);
        // double max_allowed_velocity = max_speed * max_speed_to_allowed_ratio;

        if (max_speed_to_allowed_ratio < 1) {

            return new ChassisSpeeds(
                    inputSpeeds.vxMetersPerSecond * max_speed_to_allowed_ratio,
                    inputSpeeds.vyMetersPerSecond * max_speed_to_allowed_ratio,
                    (!maintainRotationProportion)
                            ? inputSpeeds.omegaRadiansPerSecond
                            : (inputSpeeds.omegaRadiansPerSecond * max_speed_to_allowed_ratio));
        }

        return inputSpeeds;
    }

    private final AntiTipSlewer antiTipSlewer = new AntiTipSlewer();

    public ChassisSpeeds limitRobotRelativeAcceleration(ChassisSpeeds speeds) {
        antiTipSlewer.setFwdXRateLimit(antiTipSlewer.getMaxAllowedAccelerationDirectional(1, true));
        antiTipSlewer.setRevXRateLimit(
                antiTipSlewer.getMaxAllowedAccelerationDirectional(-1, true));
        antiTipSlewer.setFwdYRateLimit(
                antiTipSlewer.getMaxAllowedAccelerationDirectional(1, false));
        antiTipSlewer.setRevYRateLimit(
                antiTipSlewer.getMaxAllowedAccelerationDirectional(-1, false));

        return antiTipSlewer.limitSpeeds(speeds, getState().Pose.getRotation());
    }

    //
    // The desired robot-relative speeds
    // returns the module states where robot can drive while obeying physics and not slipping
    public SwerveRequest driveRobotRelative(ChassisSpeeds speeds) {
        m_swerveState = SwerveState.ROBOT_RELATIVE;
        return m_applyRobotSpeeds.withSpeeds(speeds);
    }

    public SwerveRequest driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        m_swerveState = SwerveState.ROBOT_RELATIVE_FFW;
        return m_applyRobotSpeeds
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons());
    }

    public SwerveRequest driveFieldRelative(ChassisSpeeds speeds) {
        m_swerveState = SwerveState.FIELD_RELATIVE;
        return m_applyFieldSpeeds.withSpeeds(speeds);
    }

    public SwerveRequest driveFieldRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        m_swerveState = SwerveState.FIELD_RELATIVE_FFW;
        return m_applyFieldSpeeds
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons());
    }

    public SwerveRequest driveDriverRelative(ChassisSpeeds speeds) {
        m_swerveState = SwerveState.DRIVER_RELATIVE;
        return m_applyDriverSpeeds.withSpeeds(speeds);
    }

    public SwerveRequest driveDriverRelativeOrbit(ChassisSpeeds speeds) {
        m_swerveState = SwerveState.DRIVER_RELATIVE_ORBIT;
        return m_applyFieldSpeedsOrbit.withChassisSpeeds(speeds);
    }

    //     public SwerveRequest driveFieldRelativeNoSetpointGenerator(ChassisSpeeds speeds) {
    //         ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getState().Pose.getRotation());
    //         return driveRobotRelative()
    //     }

    //     public SwerveRequest driveWithFeedforwards(
    //             ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    //         return m_applyRobotSpeeds
    //                 .withSpeeds(speeds)
    //                 .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
    //                 .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons());
    //     }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {

        m_odometry_custom.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return defer(() -> m_sysIdRoutineToApply.quasistatic(direction));
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return defer(() -> m_sysIdRoutineToApply.dynamic(direction));
    }

    public Command sysIdTranslationMode() {
        return Commands.runOnce(() -> m_sysIdRoutineToApply = m_sysIdRoutineTranslation);
    }

    public Command sysIdSteerMode() {
        return Commands.runOnce(() -> m_sysIdRoutineToApply = m_sysIdRoutineSteer);
    }

    public Command sysIdRotationMode() {
        return Commands.runOnce(() -> m_sysIdRoutineToApply = m_sysIdRoutineRotation);
    }

    private double lastSpeed = 0;

    private double getMaxAcceleration(double comHeight, double comX) {
        throw new RuntimeException();
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            allianceColor -> {
                                m_drivetrain.setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? kRedAlliancePerspectiveRotation
                                                : kBlueAlliancePerspectiveRotation);
                                m_hasAppliedOperatorPerspective = true;
                            });
        }

        {
            antiTipSlewer.setFwdXRateLimit(1);
            antiTipSlewer.setRevXRateLimit(1);
            antiTipSlewer.setFwdYRateLimit(1);
            antiTipSlewer.setRevYRateLimit(1);
        }

        Logger.recordOutput(
                "Drive/desiredChassisSpeeds", m_applyFieldSpeedsOrbit.getChassisSpeeds());
        Logger.recordOutput(
                "Drive/slewedChassisSpeeds", m_applyFieldSpeedsOrbit.getSlewedFieldChassisSpeeds());
        Logger.recordOutput(
                "Drive/setpointChassisSpeeds",
                m_applyFieldSpeedsOrbit.getPreviousSetpoint().robotRelativeSpeeds());

        ChassisSpeeds speedsPreview = getChassisSpeeds();

        double currentSpeed =
                Math.hypot(speedsPreview.vxMetersPerSecond, speedsPreview.vyMetersPerSecond);
        double acceleration = (currentSpeed - lastSpeed) / 0.02;
        lastSpeed = currentSpeed;

        Logger.recordOutput("Drive/Velocity", currentSpeed);
        Logger.recordOutput("Drive/Acceleration", acceleration);

        Logger.recordOutput(
                "Drive/NotZero", m_applyFieldSpeedsOrbit.getChassisSpeeds().vxMetersPerSecond != 0);

        Logger.recordOutput("Drive/ModuleTargets", getState().ModuleTargets);
        Logger.recordOutput("Drive/ModulePositions", getState().ModulePositions);
        Logger.recordOutput("Drive/ModuleStates", getState().ModuleStates);

        double[] driveMotorStatorCurrents = new double[4];
        double[] driveMotorSupplyCurrents = new double[4];
        double[] turnMotorStatorCurrents = new double[4];
        double[] turnMotorSupplyCurrents = new double[4];

        double[] driveMotorVoltage = new double[4];
        double[] turnMotorVoltage = new double[4];

        m_field2d.setRobotPose(getRobotPose());

        for (int i = 0; i < 4; i++) {
            var module = m_drivetrain.getModule(i);
            driveMotorStatorCurrents[i] =
                    module.getDriveMotor().getStatorCurrent().refresh().getValueAsDouble();
            driveMotorSupplyCurrents[i] =
                    module.getDriveMotor().getSupplyCurrent().refresh().getValueAsDouble();
            turnMotorStatorCurrents[i] =
                    module.getSteerMotor().getStatorCurrent().refresh().getValueAsDouble();
            turnMotorSupplyCurrents[i] =
                    module.getSteerMotor().getSupplyCurrent().refresh().getValueAsDouble();
            driveMotorVoltage[i] =
                    module.getDriveMotor().getMotorVoltage().refresh().getValueAsDouble();
            turnMotorVoltage[i] =
                    module.getSteerMotor().getMotorVoltage().refresh().getValueAsDouble();
        }

        Logger.recordOutput("Drive/Modules/DriveStatorCurrents", driveMotorStatorCurrents);
        Logger.recordOutput("Drive/Modules/DriveSupplyCurrents", driveMotorSupplyCurrents);

        Logger.recordOutput("Drive/Modules/TurnStatorCurrent", turnMotorStatorCurrents);
        Logger.recordOutput("Drive/Modules/TurnSupplyCurrent", turnMotorSupplyCurrents);

        Logger.recordOutput("Drive/Modules/DriveStatorVoltage", driveMotorVoltage);
        Logger.recordOutput("Drive/Modules/TurnStatorVoltage", turnMotorVoltage);

        Logger.recordOutput("Drive/actualChassisSpeeds", getState().Speeds);

        Logger.recordOutput("Drive/pose", getState().Pose);

        Logger.recordOutput("Drive/outdatedPose", m_drivetrain.getState().Pose);

        Logger.recordOutput("Drive/currentAction", m_swerveState.label);
        //      ////Logger.recordOutput("Drive/fieldPose", m_field2d);

        Logger.recordOutput("Drive/slippingModule", m_odometry_custom.m_maxSlippingWheelIndex);

        Logger.recordOutput("Drive/slippingModuleAmount", m_odometry_custom.m_maxSlippingAmount);
        Logger.recordOutput("Drive/slippingModuleRatio", m_odometry_custom.m_maxSlippingRatio);

        Logger.recordOutput("Drive/customOdometryTime", m_odometry_custom.m_lastOdometryTime);

        Logger.recordOutput("Drive/isSlipping", m_odometry_custom.m_isSlipping);
        Logger.recordOutput("Drive/isMultiSlipping", m_odometry_custom.m_isMultiwheelSlipping);

        Logger.recordOutput("Drive/translationalStandardDeviation", m_odometry_custom.m_xyVariance);

        Logger.recordOutput("Drive/rotationalStandardDeviation", m_odometry_custom.m_thetaVariance);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - m_lastSimTime;
                            m_lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            m_drivetrain.updateSimState(
                                    deltaTime, RobotController.getBatteryVoltage());
                        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void resetPose(Pose2d pose) {
        m_odometry_custom.addVisionMeasurement(
                pose, Utils.getCurrentTimeSeconds(), VecBuilder.fill(0.0, 0.0, 0.0));
        if (Robot.isSimulation()) {
            // m_drivetrain.resetPose(pose);
        }
    }

    public void setControl(SwerveRequest request) {
        m_drivetrain.setControl(request);
    }

    public void seedFieldCentric() {
        m_odometry_custom.addVisionMeasurement(
                new Pose2d(0, 0, m_drivetrain.getOperatorForwardDirection()),
                Utils.getCurrentTimeSeconds(),
                VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0));
        if (Robot.isSimulation()) {
            // m_drivetrain.seedFieldCentric();
        }
    }
}
