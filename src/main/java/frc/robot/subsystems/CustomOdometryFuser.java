package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.CircularBuffer;
import java.util.Optional;

public class CustomOdometryFuser {
    private static record TimedInfo(
            double poseX,
            double poseY,
            double poseTheta,
            double xyVariance,
            double thetaVariance,
            double timestamp) {}

    CircularBuffer<TimedInfo> timedInfoBuffer = new CircularBuffer<>(100);
    double translationVarianceDetractor = 0.0;
    double rotationVarianceDetractor = 0.0;
    double poseXOffset = 0.0;
    double poseYOffset = 0.0;
    double poseThetaOffset = 0.0;
    double poseThetaOffsetCos = 1.0;
    double poseThetaOffsetSin = 0.0;

    public CustomOdometryFuser() {}

    public void resetPose(Pose2d newPose, double timestamp) {
        timedInfoBuffer.clear();
        timedInfoBuffer.addLast(
                new TimedInfo(
                        newPose.getX(),
                        newPose.getY(),
                        newPose.getRotation().getRadians(),
                        0.0,
                        0.0,
                        timestamp));
        translationVarianceDetractor = 0.0;
        rotationVarianceDetractor = 0.0;
        poseXOffset = 0.0;
        poseYOffset = 0.0;
        poseThetaOffset = 0.0;
        poseThetaOffsetCos = 1.0;
        poseThetaOffsetSin = 0.0;
    }

    public Pose2d getPose() {
        final var tInfo = timedInfoBuffer.getLast();

        double currentPoseX =
                tInfo.poseX * poseThetaOffsetCos
                        - tInfo.poseY * poseThetaOffsetSin
                        + poseXOffset;
        double currentPoseY =
                tInfo.poseY * poseThetaOffsetSin
                        + tInfo.poseX * poseThetaOffsetCos
                        + poseYOffset;
        double currentPoseTheta = tInfo.poseTheta + poseThetaOffset;
        return new Pose2d(currentPoseX, currentPoseY, new Rotation2d(currentPoseTheta));
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        int timestampIndex = getTimestampIndex(timestamp);

        if (timestampIndex == -1) {
            return Optional.empty();
        }

        final var tInfo = timedInfoBuffer.get(timestampIndex);

        double currentPoseX =
                tInfo.poseX * poseThetaOffsetCos
                        - tInfo.poseY * poseThetaOffsetSin
                        + poseXOffset;
        double currentPoseY =
                tInfo.poseX * poseThetaOffsetSin
                        + tInfo.poseY * poseThetaOffsetCos
                        + poseYOffset;
        double currentPoseTheta = tInfo.poseTheta + poseThetaOffset;
        return Optional.of(
                new Pose2d(currentPoseX, currentPoseY, new Rotation2d(currentPoseTheta)));
    }

    public double getPhysicalPoseVariance() {
        return timedInfoBuffer.getLast().xyVariance;
    }

    public double getPhysicalPoseStdDev() {
        return Math.sqrt(getPhysicalPoseVariance());
    }

    public double getRotationalPoseVariance() {
        return timedInfoBuffer.getLast().thetaVariance;
    }

    public double getRotationalPoseStdDev() {
        return Math.sqrt(getRotationalPoseVariance());
    }

    private final Pose2d m_expPoseZero = new Pose2d();

    // You can use this to add a new measurement to the odometry fuser using this.
    public void addSwerveMeasurementTwist(
            Twist2d twist, double timestamp, double translationVariance, double rotationVariance) {
        final Pose2d postExp = m_expPoseZero.exp(twist);
        double dx = postExp.getX();
        double dy = postExp.getY();
        double dtheta = postExp.getRotation().getRadians();

        final var tInfo = timedInfoBuffer.getLast();

        double dt = timestamp - tInfo.timestamp;

        timedInfoBuffer.addLast(new TimedInfo(tInfo.poseX + dx,
        tInfo.poseY + dy,
        tInfo.poseTheta + dtheta,

       
                tInfo.xyVariance + translationVariance * dt,
        
                tInfo.thetaVariance + rotationVariance * dt,

        timestamp));
    }

    private int getTimestampIndex(double timestamp) {
        return findLargestDoubleLessThan(timedInfoBuffer, timestamp);
    }

    private static int findLargestDoubleLessThan(CircularBuffer<TimedInfo> array, double target) {
        int left = 0;
        int right = array.size() - 1;
        int result = -1; // To indicate no valid result if not found

        while (left <= right) {
            int mid = left + (right - left) / 2;

            if (array.get(mid).timestamp < target) {
                result = mid; // Update result since array[mid] is a valid candidate
                left = mid + 1; // Move to the right to find a larger candidate
            } else {
                right = mid - 1; // Move to the left to find smaller values
            }
        }

        return result;
    }

    private double m_lastVisionUpdateTimestamp;

    // Use this to add a vision pose update. This will update the odometry fuser with the vision
    // pose and the timestamp. At the moment this can only take vision pose estimates in time order.
    // AKA If you have multiple cameras, they might not both get processed depending on your
    // staggering. Think some thunks about this.
    public void addVisionUpdate(
            Pose2d visionPose,
            double timestamp,
            double translationVariance,
            double rotationVariance) {
        if (timestamp < m_lastVisionUpdateTimestamp) {
            return;
        }

        int timestampIndex = getTimestampIndex(timestamp);

        if (timestampIndex == -1) {
            return;
        }

        double visionX = visionPose.getX();
        double visionY = visionPose.getY();
        double visionTheta = visionPose.getRotation().getRadians();

        final var tInfo = timedInfoBuffer.get(timestampIndex);

        double currentPoseX =
                tInfo.poseX * poseThetaOffsetCos
                        - tInfo.poseY * poseThetaOffsetSin
                        + poseXOffset;
        double currentPoseY =
                tInfo.poseY * poseThetaOffsetSin
                        + tInfo.poseX * poseThetaOffsetCos
                        + poseYOffset;
        double currentPoseTheta = tInfo.poseTheta + poseThetaOffset;

        double physicalPoseVariance =
                tInfo.xyVariance + translationVarianceDetractor;
        double rotationalPoseVariance =
                tInfo.thetaVariance + rotationVarianceDetractor;

        double newcurrentPoseX =
                squareMerge(currentPoseX, physicalPoseVariance, visionX, translationVariance);
        double newcurrentPoseY =
                squareMerge(currentPoseY, physicalPoseVariance, visionY, translationVariance);
        double newcurrentPoseTheta =
                squareMerge(
                        currentPoseTheta, rotationalPoseVariance, visionTheta, rotationVariance);

        double newPhysicalPoseVariance = tMerge(physicalPoseVariance, translationVariance);
        double newRotationalPoseVariance = tMerge(rotationalPoseVariance, rotationVariance);

        translationVarianceDetractor += newPhysicalPoseVariance - physicalPoseVariance;
        rotationVarianceDetractor += newRotationalPoseVariance - rotationalPoseVariance;

        poseThetaOffset += newcurrentPoseTheta - currentPoseTheta;
        poseThetaOffsetCos = Math.cos(poseThetaOffset);
        poseThetaOffsetSin = Math.sin(poseThetaOffset);

        double recalculatedPoseX =
                tInfo.poseX * poseThetaOffsetCos
                        - tInfo.poseY * poseThetaOffsetSin
                        + poseXOffset;
        double recalculatedPoseY =
                tInfo.poseY * poseThetaOffsetSin
                        + tInfo.poseX * poseThetaOffsetCos
                        + poseYOffset;

        poseXOffset += newcurrentPoseX - recalculatedPoseX;
        poseYOffset += newcurrentPoseY - recalculatedPoseY;
    }

    private double squareMerge(double a, double a_var, double b, double b_var) {
        double a2 = 1 / (a_var);
        double b2 = 1 / (b_var);
        double denom = 1 / (a2 + b2);

        return (a * a2 + b * b2) / denom;
    }

    private double tMerge(double a_var, double b_var) {
        return 1 / (1 / a_var + 1 / b_var);
    }
}
