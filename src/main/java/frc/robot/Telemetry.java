package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();
    }

    double[] odometry = new double[3];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {

        Pose2d robotPose = state.Pose;

        odometry[0] = robotPose.getX();
        odometry[1] = robotPose.getY();
        odometry[2] = robotPose.getRotation().getDegrees();

        SignalLogger.writeDoubleArray("odometry", odometry);
        SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");
    }
}
