package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

public class CustomOdometry {
    public Pose2d m_currentPose = new Pose2d();
    private SwerveModulePosition[] m_lastSwerveModulePositionsCustomOdom = new SwerveModulePosition[4];
    private final CustomInverseKinematics m_kinematics_custom;
    private final DoubleSupplier m_gyroAngularVelocitySupplier;
    final double m_slippingThreshold = 1;

    public boolean m_isSlipping = false;
    public boolean m_isMultiwheelSlipping = false;

    public SimpleMatrix m_derotatedWheelVelocities = new SimpleMatrix(8, 1);
    public int m_maxSlippingWheelIndex = -1;

    public double m_lastOdometryTime = 0;

    public CustomOdometry(CustomInverseKinematics kinematics, DoubleSupplier gyroAngularVelocitySupplier) {
        m_kinematics_custom = kinematics;
        m_gyroAngularVelocitySupplier = gyroAngularVelocitySupplier;
    }

    public CustomOdometry(CustomInverseKinematics kinematics, Pigeon2 pigeon2) {
        this(kinematics, () -> pigeon2.getAngularVelocityZWorld().refresh().getValue().in(RadiansPerSecond));
    }

    public void odometryFunction(SwerveDrivetrain.SwerveDriveState state) {
        try {
            double start = Timer.getTimestamp();

            SimpleMatrix wheel_velocities_no_rot =
                    m_kinematics_custom
                            .toModuleVelocities(state.ModuleStates)
                            .minus(
                                    m_kinematics_custom.toModuleVelocities(
                                            new ChassisSpeeds(
                                                    0,
                                                    0,
                                                    m_gyroAngularVelocitySupplier.getAsDouble())));
            
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

            boolean slipping;

            if (maxWheelError > m_slippingThreshold) {
                    slipping = true;
                    m_maxSlippingWheelIndex = maxWheelErrorIndex;
            } else {
                    slipping = false;
                    m_maxSlippingWheelIndex = -1;
            }

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
                            wheelErrors[i] = Math.hypot(wheel_velocities_no_rot.get(i * 2, 0) - xVelocity, wheel_velocities_no_rot.get(i * 2 + 1, 0) - yVelocity);
                            if (wheelErrors[i] > maxWheelError) {
                                maxWheelError = wheelErrors[i];
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

            if (m_lastSwerveModulePositionsCustomOdom[0] == null) {
                    m_lastSwerveModulePositionsCustomOdom = state.ModulePositions.clone();
            }

            if (slipping & !multiWheelSlipping) {
                    poseChange = m_kinematics_custom.toTwist2d(maxWheelErrorIndex, m_lastSwerveModulePositionsCustomOdom, state.ModulePositions);
            } else {
                    poseChange = m_kinematics_custom.toTwist2d(m_lastSwerveModulePositionsCustomOdom, state.ModulePositions);
            }

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
