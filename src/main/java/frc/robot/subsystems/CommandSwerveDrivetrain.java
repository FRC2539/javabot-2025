package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.lang.annotation.Inherited;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Optional;
import java.util.Queue;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.dyn4j.geometry.Rotation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PhoenixUtil;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private SwerveDriveSimulation m_driveSim;

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
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withDesaturateWheelSpeeds(false);

    public final SwerveRequest.ApplyFieldSpeeds m_applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    public final FieldOrientedOrbitSwerveRequest m_applyFieldSpeedsOrbit;
    RobotConfig config; // PathPlanner robot configuration

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* The Setpoint Generator */
    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public void setUpPathPlanner() {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getRobotPose,
            this::resetPose,  
            this::getChassisSpeeds, 
            (speeds, feedforwards) -> setControl(
                    m_applyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ), 
            config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public Pose2d getRobotPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.getState().Speeds;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(drivetrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     * @param visionStandardDeviation    The standard deviation for vision calculation
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_applyFieldSpeedsOrbit = generateSwerveSetpointConfig();
    }

    private FieldOrientedOrbitSwerveRequest generateSwerveSetpointConfig()
    {
        RobotConfig config;
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
          throw new RuntimeException("Failed to load robot config from pathplanner.");
        }

        setpointGenerator = new SwerveSetpointGenerator(
            config, 
            Units.rotationsToRadians(10.0) //max rotational speed
        );

        ChassisSpeeds currentSpeeds = getState().Speeds; 
        SwerveModuleState[] currentStates = getState().ModuleStates; 
        SwerveSetpoint previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));

        var request = new FieldOrientedOrbitSwerveRequest(setpointGenerator, previousSetpoint, getState().Pose.getRotation());
        request.withDriverOrientation(true);
        return request;
    }

    //
    //The desired robot-relative speeds
    //returns the module states where robot can drive while obeying physics and not slipping
    public SwerveRequest driveRobotRelative(ChassisSpeeds speeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );
        return m_applyRobotSpeeds.withSpeeds(previousSetpoint.robotRelativeSpeeds())
            .withWheelForceFeedforwardsX(previousSetpoint.feedforwards().robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(previousSetpoint.feedforwards().robotRelativeForcesYNewtons());
            // Method that will drive the robot given target module states
    }

    public SwerveRequest driveRobotRelative(double xVelocity, double yVelocity, double rotationRate) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, rotationRate);
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );
        return m_applyRobotSpeeds.withSpeeds(previousSetpoint.robotRelativeSpeeds())
            .withWheelForceFeedforwardsX(previousSetpoint.feedforwards().robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(previousSetpoint.feedforwards().robotRelativeForcesYNewtons());
            // Method that will drive the robot given target module states
    }

    public SwerveRequest driveFieldRelative(ChassisSpeeds speeds) {
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getState().Pose.getRotation());
        return driveRobotRelative(speeds);
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
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
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
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        var state = getState();

        Logger.recordOutput("Drive/desiredChassisSpeeds", m_applyFieldSpeedsOrbit.getChassisSpeeds());
        Logger.recordOutput("Drive/setpoint", m_applyFieldSpeedsOrbit.getPreviousSetpoint());


        if (Double.isNaN(state.ModuleTargets[0].speedMetersPerSecond)) {
            System.out.println("ahh");
        }
        Logger.recordOutput("Drive/desiredModuleStates", state.ModuleTargets);
        Logger.recordOutput("Drive/setpointModuleStates", state.ModuleTargets);

        Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Drive/ModulePositions", state.ModulePositions);

        Logger.recordOutput("Drive/Velocity", state.Speeds);
        Logger.recordOutput("Drive/Pose", state.Pose);
        Logger.recordOutput("Drive/Odometry Period", 1.0 / state.OdometryPeriod);
    }

    private void startSimThread() {
        DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();
        config.withGyro(() -> new GyroSimulation(0.5, 0.02) {
            public void updateSimulationSubTick(double actualAngularVelocityRadPerSec) {
                super.updateSimulationSubTick(actualAngularVelocityRadPerSec);
                CommandSwerveDrivetrain.this.getPigeon2().getSimState().setRawYaw(getGyroReading().getMeasure());
                CommandSwerveDrivetrain.this.getPigeon2().getSimState().setAngularVelocityZ(getMeasuredAngularVelocity());
                CommandSwerveDrivetrain.this.getPigeon2().getSimState().setSupplyVoltage(12.0);
            }
        });
        m_driveSim = new SwerveDriveSimulation(config, new Pose2d(2,2, new Rotation2d()));

        m_lastSimTime = Utils.getCurrentTimeSeconds();

        SimulatedArena.getInstance().addDriveTrainSimulation(m_driveSim);

        var sim_modules = m_driveSim.getModules();
        var drive_modules = getModules();

        for (int i = 0; i < 4; i++) {
            
            sim_modules[i].useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(drive_modules[i].getDriveMotor(), drive_modules[i].getDriveMotor().getInverted()));

            MagnetSensorConfigs configCanCoder = new MagnetSensorConfigs();

            drive_modules[i].getCANcoder().getConfigurator().refresh(configCanCoder);
            ;
            
            sim_modules[i].useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
                drive_modules[i].getSteerMotor(),
                drive_modules[i].getSteerMotor().getInverted(),
                drive_modules[i].getCANcoder(),
                configCanCoder.SensorDirection == SensorDirectionValue.CounterClockwise_Positive,
                Rotations.of(configCanCoder.MagnetOffset)
                ));
        }

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        // m_simNotifier = new Notifier(() -> {
            
        //     final double currentTime = Utils.getCurrentTimeSeconds();
        //     double deltaTime = currentTime - m_lastSimTime;
        //     m_lastSimTime = currentTime;

        //     //updateSimState(currentTime, RobotController.getBatteryVoltage());
        //     /* use the measured time delta, get battery voltage from WPILib */
        //     // updateSimState(deltaTime, RobotController.getBatteryVoltage());
        // });
        // // m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    Rotation2d m_lastAngle = new Rotation2d();

    private void updateSimStateNew(double dtSeconds, double supplyVoltage) {
        var modulesToApply = getModules();
        
        var m_modules = m_driveSim.getModules();

        if (modulesToApply.length != m_modules.length) return;

        SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
        /* Update our sim devices */
        for (int i = 0; i < m_modules.length; ++i) {
            TalonFXSimState driveMotor = modulesToApply[i].getDriveMotor().getSimState();
            TalonFXSimState steerMotor = modulesToApply[i].getSteerMotor().getSimState();
            CANcoderSimState cancoder = modulesToApply[i].getCANcoder().getSimState();


            // getting information from the differrnt sim?
            // driveMotor.Orientation = modulesToApply[i].getDriveMotor().getInverted() ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            // steerMotor.Orientation = modulesToApply[i].getDriveMotor().getInverted() ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

            // driveMotor.setSupplyVoltage(supplyVoltage);
            // steerMotor.setSupplyVoltage(supplyVoltage);
            // cancoder.setSupplyVoltage(supplyVoltage);

            // this is going to be replaced with setting information on the different sim

            states[i] = modulesToApply[i].getCurrentState();
        }

        // var m_pigeonSim = getPigeon2().getSimState();

        // m_lastAngle = m_lastAngle.plus(Rotation2d.fromRadians(angularVelRadPerSec * dtSeconds));
        // m_pigeonSim.setRawYaw(m_lastAngle.getDegrees());
        // m_pigeonSim.setAngularVelocityZ(Units.radiansToDegrees(angularVelRadPerSec));
    }
}
