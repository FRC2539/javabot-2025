package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class FieldOrientedOrbitSwerveRequest implements SwerveRequest {
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withDesaturateWheelSpeeds(false);

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
    private ChassisSpeeds slewedFieldChassisSpeeds = new ChassisSpeeds();

    private boolean useDriverOrientation = true;

    private double forwardXRateLimit = Double.POSITIVE_INFINITY;
    private double backwardXRateLimit = Double.POSITIVE_INFINITY;

    private double forwardYRateLimit = Double.POSITIVE_INFINITY;
    private double backwardYRateLimit = Double.POSITIVE_INFINITY;

    private double timestep = 0.004;

    private boolean maintainStraightStopping = true;

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public SwerveSetpoint getPreviousSetpoint() {
        return previousSetpoint;
    }

    /**
     * @param xTipLimiter
     * @param yTipLimiter
     * @param setpointGenerator
     * @param initialSetpoint
     * 
     * This creates a FieldOrientedOrbitSwerveRequest.
     * 
     * The anti-tipping logic currently does not actually work due to rotation and is taken out right now.
     */
    public FieldOrientedOrbitSwerveRequest(SwerveSetpointGenerator setpointGenerator, SwerveSetpoint initialSetpoint, Rotation2d robotOrientation) {

        this.setpointGenerator = setpointGenerator;

        this.withPreviousSetpoint(initialSetpoint, robotOrientation);
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

        double xAcceleration = toApplyX - slewedFieldChassisSpeeds.vxMetersPerSecond;
        double yAcceleration = toApplyY - slewedFieldChassisSpeeds.vyMetersPerSecond;
        
        ChassisSpeeds accelerations = new ChassisSpeeds(xAcceleration, yAcceleration, 0);
        accelerations = ChassisSpeeds.fromFieldRelativeSpeeds(accelerations,parameters.currentPose.getRotation());

        if (accelerations.vxMetersPerSecond > forwardXRateLimit*timestep) {
            if (maintainStraightStopping) accelerations.vyMetersPerSecond = accelerations.vyMetersPerSecond * forwardXRateLimit*timestep / accelerations.vxMetersPerSecond;
            accelerations.vxMetersPerSecond = forwardXRateLimit*timestep;
        } else if (accelerations.vxMetersPerSecond < -backwardXRateLimit*timestep) {
            if (maintainStraightStopping) accelerations.vyMetersPerSecond = accelerations.vyMetersPerSecond * -backwardXRateLimit*timestep / accelerations.vxMetersPerSecond;
            accelerations.vxMetersPerSecond = -backwardXRateLimit*timestep;
        }

        if (accelerations.vyMetersPerSecond > forwardYRateLimit*timestep) {
            if (maintainStraightStopping) accelerations.vyMetersPerSecond = accelerations.vxMetersPerSecond * forwardYRateLimit*timestep / accelerations.vyMetersPerSecond;
            accelerations.vyMetersPerSecond = forwardYRateLimit*timestep;
        } else if (accelerations.vyMetersPerSecond < -backwardYRateLimit*timestep) {
            if (maintainStraightStopping) accelerations.vxMetersPerSecond = accelerations.vxMetersPerSecond * -backwardYRateLimit*timestep / accelerations.vyMetersPerSecond;
            accelerations.vyMetersPerSecond = -backwardYRateLimit*timestep;
        }

        accelerations = ChassisSpeeds.fromFieldRelativeSpeeds(accelerations,parameters.currentPose.getRotation());

        slewedFieldChassisSpeeds = slewedFieldChassisSpeeds.plus(accelerations);

        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(slewedFieldChassisSpeeds.vxMetersPerSecond, slewedFieldChassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);

        robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds,parameters.currentPose.getRotation());

        // Keep the robot from tipping over
        // robotRelativeSpeeds.vxMetersPerSecond = xLimiter.calculate(robotRelativeSpeeds.vxMetersPerSecond);
        // robotRelativeSpeeds.vyMetersPerSecond = yLimiter.calculate(robotRelativeSpeeds.vyMetersPerSecond);

        // Apply all other limits
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, robotRelativeSpeeds, timestep);

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

    public FieldOrientedOrbitSwerveRequest withPreviousSetpoint(SwerveSetpoint previousSetpoint, Rotation2d robotOrientation) {
        this.previousSetpoint = previousSetpoint;
        this.slewedFieldChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotOrientation);
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withDriverOrientation(boolean useDriverOrientation) {
        this.useDriverOrientation = useDriverOrientation;
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withXRateLimits(double forwardXRateLimit, double backwardXRateLimit) {
        this.forwardXRateLimit = forwardXRateLimit;
        this.backwardXRateLimit = backwardXRateLimit;
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withYRateLimits(double forwardYRateLimit, double backwardYRateLimit) {
        this.forwardYRateLimit = forwardYRateLimit;
        this.backwardYRateLimit = backwardYRateLimit;
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withTimestep(double timestep) {
        this.timestep = timestep;
        return this;
    }

    public FieldOrientedOrbitSwerveRequest withMaintainStraightStopping(boolean maintainStraightStopping) {
        this.maintainStraightStopping = maintainStraightStopping;
        return this;
    }
}