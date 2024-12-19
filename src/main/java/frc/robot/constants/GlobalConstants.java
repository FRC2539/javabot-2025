package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class GlobalConstants {
    
    private static final SwerveModuleConstants EXAMPLE_MODULE = TunerConstants.FrontLeft;

    public static final LinearVelocity MAX_TRANSLATIONAL_SPEED = TunerConstants.kSpeedAt12Volts;
    public static  final AngularVelocity MAX_ROTATIONAL_SPEED = RotationsPerSecond.of(1);

    public static final Mass ROBOT_MASS = Pounds.of(150);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);

    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1).withReduction(EXAMPLE_MODULE.DriveMotorGearRatio);
    public static final DCMotor STEER_MOTOR = DCMotor.getKrakenX60Foc(1).withReduction(EXAMPLE_MODULE.SteerMotorGearRatio);

    public static class ControllerConstants {
        public static final int LEFT_DRIVE_CONTROLLER = 0;
        public static final int RIGHT_DRIVE_CONTROLLER = 1;
        public static final int OPERATOR_CONTROLLER = 2;
    }

    private static RobotConfig robotConfigPathplanner;

    public static RobotConfig getRobotConfigPathplanner() {
        if (robotConfigPathplanner == null) {
            try{
              Translation2d[] moduleOffsets = new Translation2d[4];
              SwerveModuleConstants[] constants = new SwerveModuleConstants[]{
                TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
              };
              for (int i = 0; i < constants.length; i++) {
                  moduleOffsets[i] = new Translation2d(constants[i].LocationX, constants[i].LocationY);
              }

              robotConfigPathplanner = new RobotConfig(ROBOT_MASS, ROBOT_MOI, 
                new ModuleConfig(
                    Meters.of(EXAMPLE_MODULE.WheelRadius), 
                    MAX_TRANSLATIONAL_SPEED, 1, 
                    DRIVE_MOTOR, EXAMPLE_MODULE.DriveMotorInitialConfigs.CurrentLimits.getStatorCurrentLimitMeasure(),
                    1
                ),
                moduleOffsets);
            } catch (Exception e) {
              // Handle exception as needed
              e.printStackTrace();
              throw new RuntimeException("Failed to load robot config from pathplanner.");
            }
        }
        return robotConfigPathplanner;
    }
}
