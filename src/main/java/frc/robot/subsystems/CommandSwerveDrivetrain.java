package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final CustomInverseKinematics m_kinematics_custom;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
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
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                    .withDesaturateWheelSpeeds(false);

    public final SwerveRequest.ApplyFieldSpeeds m_applyFieldSpeeds =
            new SwerveRequest.ApplyFieldSpeeds();

    public final FieldOrientedOrbitSwerveRequest m_applyFieldSpeedsOrbit;
    RobotConfig config; // PathPlanner robot configuration

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

    public void setUpPathPlanner() {}

    public Pose2d getRobotPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.getState().Speeds;
    }

    private SwerveModulePosition[] m_lastSwerveModulePositionsCustomOdom = new SwerveModulePosition[4];
    private void customOdometryStep(SwerveDrivetrain.SwerveDriveState state) {
        double start = Utils.getCurrentTimeSeconds();
        final double slippingThreshold = 1;

        SimpleMatrix wheel_velocities_no_rot =
                m_kinematics_custom
                        .toModuleVelocities(state.ModuleStates)
                        .minus(
                                m_kinematics_custom.toModuleVelocities(
                                        new ChassisSpeeds(
                                                0,
                                                0,
                                                getPigeon2()
                                                        .getAngularVelocityZWorld()
                                                        .refresh()
                                                        .getValue()
                                                        .in(RadiansPerSecond))));
        Logger.recordOutput(
                "Drive/RotationlessVectors", wheel_velocities_no_rot.transpose().toArray2()[0]);
        
        double xVelocity = 0;
        double yVelocity = 0;
        for (int i = 0; i < 4; i++) {
            xVelocity += wheel_velocities_no_rot.get(i * 2, 0);
            yVelocity += wheel_velocities_no_rot.get(i * 2 + 1, 0);
        }
        xVelocity /= 4;
        yVelocity /= 4;

        double[] wheelErrors = new double[4];

        double maxWheelError = 0;
        int maxWheelErrorIndex = 0;

        for (int i = 0; i < 4; i++) {
            wheelErrors[i] = Math.hypot(wheel_velocities_no_rot.get(i * 2, 0) - xVelocity, wheel_velocities_no_rot.get(i * 2 + 1, 0) - yVelocity);
            if (wheelErrors[i] > maxWheelError) {
                maxWheelError = wheelErrors[i];
                maxWheelErrorIndex = i;
            }
        }

        Logger.recordOutput(
                "Drive/SlippingAmount", wheelErrors);

        boolean slipping;

        if (maxWheelError > slippingThreshold) {
                slipping = true;
                Logger.recordOutput("Drive/MaxSlippingWheel", maxWheelErrorIndex);
        } else {
                slipping = false;
                Logger.recordOutput("Drive/MaxSlippingWheel", -1);
        }

        Logger.recordOutput("Drive/Slipping", slipping);

        boolean multiWheelSlipping = false;

        if (slipping) {
                xVelocity = 0;
                yVelocity = 0;
                for (int i = 0; i < 3; i++) {
                        int j = i;
                         if (i >= maxWheelErrorIndex) {
                                j++;
                        }
                    xVelocity += wheel_velocities_no_rot.get(j * 2, 0);
                    yVelocity += wheel_velocities_no_rot.get(j * 2 + 1, 0);
                }
                xVelocity /= 3;
                yVelocity /= 3;

                maxWheelError = 0;

                for (int i = 0; i < 3; i++) {
                        wheelErrors[i] = Math.hypot(wheel_velocities_no_rot.get(i * 2, 0) - xVelocity, wheel_velocities_no_rot.get(i * 2 + 1, 0) - yVelocity);
                        if (wheelErrors[i] > maxWheelError) {
                            maxWheelError = wheelErrors[i];
                        }
                }

                if (maxWheelError > slippingThreshold) {
                        multiWheelSlipping = true;
                }
        }

        Logger.recordOutput("Drive/MultiWheelSlipping", multiWheelSlipping);

        Twist2d poseChange;
        double translationStds;
        double rotationStds;

        if (slipping & !multiWheelSlipping) {
                poseChange = m_kinematics_custom.toTwist2d(maxWheelErrorIndex, state.ModulePositions, m_lastSwerveModulePositionsCustomOdom);
        } else {
                poseChange = m_kinematics_custom.toTwist2d(state.ModulePositions, m_lastSwerveModulePositionsCustomOdom);
        }

        m_lastSwerveModulePositionsCustomOdom = state.ModulePositions;




        double end = Utils.getCurrentTimeSeconds();
        Logger.recordOutput("Drive/CustomOdometryTime", end - start);
    }

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
            SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();
        m_kinematics_custom = new CustomInverseKinematics(getModuleLocations());
        registerTelemetry(this::customOdometryStep);
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
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(drivetrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();
        m_kinematics_custom = new CustomInverseKinematics(getModuleLocations());
        registerTelemetry(this::customOdometryStep);
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
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     * @param visionStandardDeviation The standard deviation for vision calculation
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();
        m_kinematics_custom = new CustomInverseKinematics(getModuleLocations());
        registerTelemetry(this::customOdometryStep);
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

    //
    // The desired robot-relative speeds
    // returns the module states where robot can drive while obeying physics and not slipping
    public SwerveRequest driveRobotRelative(ChassisSpeeds speeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint =
                setpointGenerator.generateSetpoint(
                        previousSetpoint, // The previous setpoint
                        speeds, // The desired target speeds
                        0.02 // The loop time of the robot code, in seconds
                        );
        return m_applyRobotSpeeds
                .withSpeeds(previousSetpoint.robotRelativeSpeeds())
                .withWheelForceFeedforwardsX(
                        previousSetpoint.feedforwards().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(
                        previousSetpoint.feedforwards().robotRelativeForcesYNewtons());
        // Method that will drive the robot given target module states
    }

    public SwerveRequest driveRobotRelative(
            double xVelocity, double yVelocity, double rotationRate) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, rotationRate);
        return m_applyFieldSpeedsOrbit.withChassisSpeeds(speeds);
        // Method that will drive the robot given target module states
    }

    public SwerveRequest driveFieldRelative(ChassisSpeeds speeds) {
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getState().Pose.getRotation());
        return driveRobotRelative(speeds);
    }

    public SwerveRequest driveWithFeedforwards(
            ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        return m_applyRobotSpeeds
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons());
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private double lastSpeed = 0;

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
                                setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? kRedAlliancePerspectiveRotation
                                                : kBlueAlliancePerspectiveRotation);
                                m_hasAppliedOperatorPerspective = true;
                            });
        }

        Logger.recordOutput(
                "Drive/desiredChassisSpeeds", m_applyFieldSpeedsOrbit.getChassisSpeeds());
        Logger.recordOutput(
                "Drive/slewedChassisSpeeds", m_applyFieldSpeedsOrbit.getSlewedFieldChassisSpeeds());
        Logger.recordOutput(
                "Drive/setpointChassisSpeeds",
                m_applyFieldSpeedsOrbit.getPreviousSetpoint().robotRelativeSpeeds());

        ChassisSpeeds speedsPreview =
                m_applyFieldSpeedsOrbit.getPreviousSetpoint().robotRelativeSpeeds();

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

        for (int i = 0; i < 4; i++) {
            var module = getModule(i);
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
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
