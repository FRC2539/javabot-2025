package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldOrientedOrbitSwerveRequest implements SwerveRequest {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withDesaturateWheelSpeeds(false);

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private double[] wheelForceFeedforwardsX = new double[4];
    private double[] wheelForceFeedforwardsY = new double[4];

    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public FieldOrientedOrbitSwerveRequest(SlewRateLimiter xTipLimiter, SlewRateLimiter yTipLimiter, SwerveSetpointGenerator setpointGenerator, SwerveSetpoint initialSetpoint) {
        this.xLimiter = xTipLimiter;
        this.yLimiter = yTipLimiter;

        this.setpointGenerator = setpointGenerator;
        this.previousSetpoint = initialSetpoint;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        double toApplyX = chassisSpeeds.vxMetersPerSecond;
        double toApplyY = chassisSpeeds.vyMetersPerSecond;

        // Translation2d tmp = new Translation2d(toApplyX, toApplyY);
        // tmp = tmp.rotateBy(parameters.operatorForwardDirection);
        // toApplyX = tmp.getX();
        // toApplyY = tmp.getY();

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(toApplyX, toApplyY, chassisSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, parameters.currentPose.getRotation());

        // Keep the robot from tipping over
        robotRelativeSpeeds.vxMetersPerSecond = xLimiter.calculate(robotRelativeSpeeds.vxMetersPerSecond);
        robotRelativeSpeeds.vyMetersPerSecond = yLimiter.calculate(robotRelativeSpeeds.vyMetersPerSecond);

        // Apply all other limits
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, robotRelativeSpeeds, 0.02);

        return applyRobotSpeeds
            .withSpeeds(previousSetpoint.robotRelativeSpeeds())
            .withWheelForceFeedforwardsX(wheelForceFeedforwardsX)
            .withWheelForceFeedforwardsY(wheelForceFeedforwardsY)
            .apply(parameters, modulesToApply);
    }

    public FieldOrientedOrbitSwerveRequest withChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withWheelForceFeedforwardsX(double[] newWheelForceFeedforwardsX) {
        wheelForceFeedforwardsX = newWheelForceFeedforwardsX;
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withWheelForceFeedforwardsY(double[] newWheelForceFeedforwardsY) {
        wheelForceFeedforwardsY = newWheelForceFeedforwardsY;
        return this;
    }
}