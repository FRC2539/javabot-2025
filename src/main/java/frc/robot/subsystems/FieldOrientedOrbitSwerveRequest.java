package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private boolean useDriverOrientation;

    /**
     * @param xTipLimiter
     * @param yTipLimiter
     * @param setpointGenerator
     * @param initialSetpoint
     * 
     * This creates a FieldOrientedOrbitSwerveRequest
     */
    public FieldOrientedOrbitSwerveRequest(SlewRateLimiter xTipLimiter, SlewRateLimiter yTipLimiter, SwerveSetpointGenerator setpointGenerator, SwerveSetpoint initialSetpoint) {
        this.xLimiter = xTipLimiter;
        this.yLimiter = yTipLimiter;

        this.setpointGenerator = setpointGenerator;

        this.withPreviousSetpoint(initialSetpoint);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        double toApplyX = chassisSpeeds.vxMetersPerSecond;
        double toApplyY = chassisSpeeds.vyMetersPerSecond;

        if (useDriverOrientation) {
            Translation2d tmp = new Translation2d(toApplyX, toApplyY);
            tmp = tmp.rotateBy(parameters.operatorForwardDirection);
            toApplyX = tmp.getX();
            toApplyY = tmp.getY();
        }

        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(toApplyX, toApplyY, chassisSpeeds.omegaRadiansPerSecond);
        robotRelativeSpeeds.toRobotRelativeSpeeds(parameters.currentPose.getRotation());

        // Keep the robot from tipping over
        robotRelativeSpeeds.vxMetersPerSecond = xLimiter.calculate(robotRelativeSpeeds.vxMetersPerSecond);
        robotRelativeSpeeds.vyMetersPerSecond = yLimiter.calculate(robotRelativeSpeeds.vyMetersPerSecond);

        // Apply all other limits
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, robotRelativeSpeeds, 0.02);

        DriveFeedforwards feedforwards = previousSetpoint.feedforwards();

        return applyRobotSpeeds
            .withSpeeds(previousSetpoint.robotRelativeSpeeds())
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            .apply(parameters, modulesToApply);
    }

    public FieldOrientedOrbitSwerveRequest withChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withPreviousSetpoint(SwerveSetpoint previousSetpoint) {
        this.previousSetpoint = previousSetpoint;
        ChassisSpeeds previousRobotRelativeSpeeds = this.previousSetpoint.robotRelativeSpeeds();
        xLimiter.reset(previousRobotRelativeSpeeds.vxMetersPerSecond);
        yLimiter.reset(previousRobotRelativeSpeeds.vyMetersPerSecond);
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withDriverOrientation(boolean useDriverOrientation) {
        this.useDriverOrientation = useDriverOrientation;
        return this;
    }
}