package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import org.ejml.simple.SimpleMatrix;

public class CustomOdometry {
    public Pose2d m_currentPose = new Pose2d();
    private SwerveModulePosition[] m_lastSwerveModulePositionsCustomOdom =
            new SwerveModulePosition[4];
    private final CustomInverseKinematics m_kinematics_custom;

    final double m_slippingThreshold = 0.02;

    public boolean m_isSlipping = false;
    public boolean m_isMultiwheelSlipping = false;

    public SimpleMatrix m_derotatedWheelVelocities = new SimpleMatrix(8, 1);
    public int m_maxSlippingWheelIndex = -1;

    public double m_lastOdometryTime = 0;

    public double m_maxSlippingAmount = 0;
    public double m_maxSlippingRatio = 1;

    private double lastGryoTheta = 0;

    private boolean hasIteratedYet = false;

    public CustomOdometry(
            CustomInverseKinematics kinematics) {
        m_kinematics_custom = kinematics;
    }

    public void odometryFunction(SwerveDrivetrain.SwerveDriveState state) {
        try {
            double start = Timer.getTimestamp();

            if (!hasIteratedYet) {
                m_lastSwerveModulePositionsCustomOdom = state.ModulePositions.clone();
                lastGryoTheta = state.Pose.getRotation().getRadians();
            }

            SimpleMatrix wheel_velocities_no_rot =
                    m_kinematics_custom
                            .toModuleVelocities(state.ModuleStates)
                            .minus(
                                    m_kinematics_custom.toModuleVelocities(
                                            new ChassisSpeeds(
                                                    0,
                                                    0,
                                                    state.Speeds.omegaRadiansPerSecond)));

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
                wheelErrors[i] =
                        Math.hypot(
                                wheel_velocities_no_rot.get(i * 2, 0) - xVelocity,
                                wheel_velocities_no_rot.get(i * 2 + 1, 0) - yVelocity);
                if (wheelErrors[i] > maxWheelError) {
                    maxWheelError = wheelErrors[i];
                    maxWheelErrorIndex = i;
                }
            }

            boolean slipping;

            if (maxWheelError > m_slippingThreshold) {
                slipping = true;
            } else {
                slipping = false;
            }

            m_maxSlippingAmount = maxWheelError;

            m_maxSlippingRatio = maxWheelError / Math.hypot(xVelocity, yVelocity);

            m_maxSlippingWheelIndex = maxWheelErrorIndex;

            m_isSlipping = slipping;

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
                    int j = i;
                    if (i >= maxWheelErrorIndex) {
                        j++;
                    }
                    wheelErrors[j] =
                            Math.hypot(
                                    wheel_velocities_no_rot.get(j * 2, 0) - xVelocity,
                                    wheel_velocities_no_rot.get(j * 2 + 1, 0) - yVelocity);
                    if (wheelErrors[j] > maxWheelError) {
                        maxWheelError = wheelErrors[j];
                    }
                }

                if (maxWheelError > m_slippingThreshold) {
                    multiWheelSlipping = true;
                }
            }

            m_isMultiwheelSlipping = multiWheelSlipping;

            Twist2d poseChange;
            double translationStds;
            double rotationStds;

            if (slipping & !multiWheelSlipping) {
                poseChange =
                        m_kinematics_custom.toTwist2d(
                                maxWheelErrorIndex,
                                m_lastSwerveModulePositionsCustomOdom,
                                state.ModulePositions);
            } else {
                poseChange =
                        m_kinematics_custom.toTwist2d(
                                m_lastSwerveModulePositionsCustomOdom, state.ModulePositions);
            }

            double currentGyroTheta = state.Pose.getRotation().getRadians();

            poseChange.dtheta = currentGyroTheta - lastGryoTheta;

            m_currentPose = m_currentPose.exp(poseChange);

            m_lastSwerveModulePositionsCustomOdom = state.ModulePositions.clone();

            double end = Timer.getTimestamp();
            m_lastOdometryTime = end - start;
        } catch (Exception e) {
            System.out.println("Error in odometryFunction: " + e);
            e.printStackTrace();
        }
    }
}
