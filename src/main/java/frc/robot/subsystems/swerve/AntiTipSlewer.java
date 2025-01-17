package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AntiTipSlewer {
    private ChassisSpeeds slewedFieldChassisSpeeds = new ChassisSpeeds();

    private double forwardXRateLimit = 1;
    private double backwardXRateLimit = 1;

    private double forwardYRateLimit = 1;
    private double backwardYRateLimit = 1;


    public void setFwdXRateLimit(double fwdXRateLimit)
    {
        forwardXRateLimit = fwdXRateLimit;
    }

    public void setRevXRateLimit(double revXRateLimit)
    {
        backwardXRateLimit = revXRateLimit;
    }

    public void setFwdYRateLimit(double fwdYRateLimit)
    {
        forwardYRateLimit = fwdYRateLimit;
    }

    public void setRevYRateLimit(double revYRateLimit)
    {
        backwardYRateLimit = revYRateLimit;
    }


    // private final boolean maintainStraightStopping = false;

    private final double timestep = 0.02;

    public void resetSpeeds(ChassisSpeeds slewedFieldChassisSpeeds) {
        this.slewedFieldChassisSpeeds = slewedFieldChassisSpeeds;
    }

    public ChassisSpeeds limitSpeeds(ChassisSpeeds commandedSpeeds, Rotation2d rotation) {
        var chassisSpeeds = commandedSpeeds;
        double toApplyX = chassisSpeeds.vxMetersPerSecond;
        double toApplyY = chassisSpeeds.vyMetersPerSecond;

        // if (useDriverOrientation) {
        //     Translation2d tmp = new Translation2d(toApplyX, toApplyY);
        //     tmp = tmp.rotateBy(parameters.operatorForwardDirection);
        //     toApplyX = tmp.getX();
        //     toApplyY = tmp.getY();
        // }

        double xAcceleration = toApplyX - slewedFieldChassisSpeeds.vxMetersPerSecond;
        double yAcceleration = toApplyY - slewedFieldChassisSpeeds.vyMetersPerSecond;

        ChassisSpeeds accelerations = new ChassisSpeeds(xAcceleration, yAcceleration, 0);
        accelerations =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        accelerations, rotation);

        if (accelerations.vxMetersPerSecond > forwardXRateLimit * timestep) {
            // if (maintainStraightStopping)
            //     accelerations.vyMetersPerSecond =
            //             accelerations.vyMetersPerSecond
            //                     * forwardXRateLimit
            //                     * timestep
            //                     / accelerations.vxMetersPerSecond;
            accelerations.vxMetersPerSecond = forwardXRateLimit * timestep;
        } else if (accelerations.vxMetersPerSecond < -backwardXRateLimit * timestep) {
            // if (maintainStraightStopping)
            //     accelerations.vyMetersPerSecond =
            //             accelerations.vyMetersPerSecond
            //                     * -backwardXRateLimit
            //                     * timestep
            //                     / accelerations.vxMetersPerSecond;
            accelerations.vxMetersPerSecond = -backwardXRateLimit * timestep;
        }

        if (accelerations.vyMetersPerSecond > forwardYRateLimit * timestep) {
            // if (maintainStraightStopping)
            //     accelerations.vyMetersPerSecond =
            //             accelerations.vxMetersPerSecond
            //                     * forwardYRateLimit
            //                     * timestep
            //                     / accelerations.vyMetersPerSecond;
            accelerations.vyMetersPerSecond = forwardYRateLimit * timestep;
        } else if (accelerations.vyMetersPerSecond < -backwardYRateLimit * timestep) {
            // if (maintainStraightStopping)
            //     accelerations.vxMetersPerSecond =
            //             accelerations.vxMetersPerSecond
            //                     * -backwardYRateLimit
            //                     * timestep
            //                     / accelerations.vyMetersPerSecond;
            accelerations.vyMetersPerSecond = -backwardYRateLimit * timestep;
        }

        accelerations =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        accelerations, rotation);

        slewedFieldChassisSpeeds = slewedFieldChassisSpeeds.plus(accelerations);

        ChassisSpeeds robotRelativeSpeeds =
                new ChassisSpeeds(
                        slewedFieldChassisSpeeds.vxMetersPerSecond,
                        slewedFieldChassisSpeeds.vyMetersPerSecond,
                        chassisSpeeds.omegaRadiansPerSecond);

        return robotRelativeSpeeds;
    }
}
