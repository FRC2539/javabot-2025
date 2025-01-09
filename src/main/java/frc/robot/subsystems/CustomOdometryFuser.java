package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.CircularBuffer;
import java.util.Optional;
import java.util.Stack;

public class CustomOdometryFuser {
    private final boolean kUtilizeTurningCorrection = false;

    private static record TimedInfo(
            double poseX,
            double poseY,
            double poseTheta,
            double xyVariance,
            double thetaVariance,
            double timestamp) {}

    private CircularBuffer<TimedInfo> m_pastPosesRelative = new CircularBuffer<>(100);

    private static record VisionUpdate(
            Pose2d visionPose,
            double timestamp,
            double translationVariance,
            double rotationVariance,
            VisionUpdateOffset visionUpdateOffset) {}

    private static record VisionUpdateOffset(
            double translationVarianceDetractor,
            double rotationVarianceDetractor,
            double poseXOffset,
            double poseYOffset,
            double poseThetaOffset,
            double poseThetaOffsetCos,
            double poseThetaOffsetSin) {
        VisionUpdateOffset(
                double translationVarianceDetractor,
                double rotationVarianceDetractor,
                double poseXOffset,
                double poseYOffset,
                double poseThetaOffset) {
            this(
                    translationVarianceDetractor,
                    rotationVarianceDetractor,
                    poseXOffset,
                    poseYOffset,
                    poseThetaOffset,
                    Math.cos(poseThetaOffset),
                    Math.sin(poseThetaOffset));
        }
    }

    private final int m_pastVisionUpdatesMaxSize = 10;
    private CircularBuffer<VisionUpdate> m_pastVisionUpdates =
            new CircularBuffer<>(m_pastVisionUpdatesMaxSize);

    private Stack<VisionUpdate> m_unaddedVisionUpdates = new Stack<>();

    public CustomOdometryFuser() {
        m_pastVisionUpdates.addLast(
                new VisionUpdate(Pose2d.kZero, 0, 0, 0, new VisionUpdateOffset(0, 0, 0, 0, 0)));
    }

    public void resetPose(Pose2d newPose, double timestamp) {
        addVisionUpdate(newPose, timestamp, 0.0, 0.0);
    }

    public Pose2d getPose() {
        final var tInfo = m_pastPosesRelative.getLast();

        final var vInfo = m_pastVisionUpdates.getLast().visionUpdateOffset;

        double currentPoseX =
                tInfo.poseX * vInfo.poseThetaOffsetCos
                        - tInfo.poseY * vInfo.poseThetaOffsetSin
                        + vInfo.poseXOffset;
        double currentPoseY =
                tInfo.poseY * vInfo.poseThetaOffsetSin
                        + tInfo.poseX * vInfo.poseThetaOffsetCos
                        + vInfo.poseYOffset;
        double currentPoseTheta = tInfo.poseTheta + vInfo.poseThetaOffset;
        return new Pose2d(currentPoseX, currentPoseY, new Rotation2d(currentPoseTheta));
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        int timestampIndex = getTimestampIndex(timestamp);

        if (timestampIndex == -1) {
            return Optional.empty();
        }

        final var tInfo = m_pastPosesRelative.get(timestampIndex);
        final var vInfo = m_pastVisionUpdates.getLast().visionUpdateOffset;

        double currentPoseX =
                tInfo.poseX * vInfo.poseThetaOffsetCos
                        - tInfo.poseY * vInfo.poseThetaOffsetSin
                        + vInfo.poseXOffset;
        double currentPoseY =
                tInfo.poseX * vInfo.poseThetaOffsetSin
                        + tInfo.poseY * vInfo.poseThetaOffsetCos
                        + vInfo.poseYOffset;
        double currentPoseTheta = tInfo.poseTheta + vInfo.poseThetaOffset;
        return Optional.of(
                new Pose2d(currentPoseX, currentPoseY, new Rotation2d(currentPoseTheta)));
    }

    public double getPhysicalPoseVariance() {
        return m_pastPosesRelative.getLast().xyVariance;
    }

    public double getPhysicalPoseStdDev() {
        return Math.sqrt(getPhysicalPoseVariance());
    }

    public double getRotationalPoseVariance() {
        return m_pastPosesRelative.getLast().thetaVariance;
    }

    public double getRotationalPoseStdDev() {
        return Math.sqrt(getRotationalPoseVariance());
    }

    // You can use this to add a new measurement to the odometry fuser using this.
    public void addSwerveMeasurementTwist(
            Twist2d twist, double timestamp, double translationVariance, double rotationVariance) {
        final Pose2d postExp = Pose2d.kZero.exp(twist);
        double dx = postExp.getX();
        double dy = postExp.getY();
        double dtheta = postExp.getRotation().getRadians();

        final var tInfo = m_pastPosesRelative.getLast();

        double dt = timestamp - tInfo.timestamp;

        // This is technically not correctly considered right now.
        if (kUtilizeTurningCorrection) {
            double nextTheta = tInfo.thetaVariance + rotationVariance * dt;
            double lengthMod = Math.exp(-0.5 * nextTheta);
            dx *= lengthMod;
            dy *= lengthMod;
            double lengthVarianceAddition = (1 - Math.exp(-nextTheta)) * (dx * dx + dy * dy);
            translationVariance += lengthVarianceAddition;
        }

        m_pastPosesRelative.addLast(
                new TimedInfo(
                        tInfo.poseX + dx,
                        tInfo.poseY + dy,
                        tInfo.poseTheta + dtheta,
                        tInfo.xyVariance + translationVariance * dt,
                        tInfo.thetaVariance + rotationVariance * dt,
                        timestamp));
    }

    private int getTimestampIndex(double timestamp) {
        return findLargestDoubleLessThan(m_pastPosesRelative, timestamp);
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

    // Use this to add a vision pose update. This will update the odometry fuser with the vision
    // pose and the timestamp. At the moment this can only take vision pose estimates in time order.
    // AKA If you have multiple cameras, they might not both get processed depending on your
    // staggering. Think some thunks about this.
    public void addVisionUpdate(
            Pose2d visionPose,
            double timestamp,
            double translationVariance,
            double rotationVariance) {

        moveVisionUpdatesToStack(
                new VisionUpdate(
                        visionPose, timestamp, translationVariance, rotationVariance, null));

        while (!m_unaddedVisionUpdates.isEmpty()) {
            moveVisionUpdateOffStack();
        }
    }

    private void moveVisionUpdateOffStack() {
        final var visUpd = m_unaddedVisionUpdates.pop();

        int timestampIndex = getTimestampIndex(visUpd.timestamp);

        double visionX = visUpd.visionPose.getX();
        double visionY = visUpd.visionPose.getY();
        double visionTheta = visUpd.visionPose.getRotation().getRadians();

        final var tInfo = m_pastPosesRelative.get(timestampIndex);
        final var vInfo = m_pastVisionUpdates.getLast().visionUpdateOffset;

        double currentPoseX =
                tInfo.poseX * vInfo.poseThetaOffsetCos
                        - tInfo.poseY * vInfo.poseThetaOffsetSin
                        + vInfo.poseXOffset;
        double currentPoseY =
                tInfo.poseY * vInfo.poseThetaOffsetSin
                        + tInfo.poseX * vInfo.poseThetaOffsetCos
                        + vInfo.poseYOffset;
        double currentPoseTheta = tInfo.poseTheta + vInfo.poseThetaOffset;

        double physicalPoseVariance = tInfo.xyVariance + vInfo.translationVarianceDetractor;
        double rotationalPoseVariance = tInfo.thetaVariance + vInfo.rotationVarianceDetractor;

        double newcurrentPoseX =
                squareMerge(
                        currentPoseX, physicalPoseVariance, visionX, visUpd.translationVariance);
        double newcurrentPoseY =
                squareMerge(
                        currentPoseY, physicalPoseVariance, visionY, visUpd.translationVariance);
        double newcurrentPoseTheta =
                squareMerge(
                        currentPoseTheta,
                        rotationalPoseVariance,
                        visionTheta,
                        visUpd.rotationVariance);

        double newPhysicalPoseVariance = tMerge(physicalPoseVariance, visUpd.translationVariance);
        double newRotationalPoseVariance = tMerge(rotationalPoseVariance, visUpd.rotationVariance);

        double translationVarianceDetractorNew =
                newPhysicalPoseVariance - physicalPoseVariance + vInfo.translationVarianceDetractor;
        double rotationVarianceDetractorNew =
                newRotationalPoseVariance
                        - rotationalPoseVariance
                        + vInfo.rotationVarianceDetractor;

        double poseThetaOffsetNew = newcurrentPoseTheta - currentPoseTheta + vInfo.poseThetaOffset;
        double poseThetaOffsetCosNew = Math.cos(vInfo.poseThetaOffset);
        double poseThetaOffsetSinNew = Math.sin(vInfo.poseThetaOffset);

        double recalculatedPoseX =
                tInfo.poseX * poseThetaOffsetCosNew
                        - tInfo.poseY * poseThetaOffsetSinNew
                        + vInfo.poseXOffset;
        double recalculatedPoseY =
                tInfo.poseY * poseThetaOffsetSinNew
                        + tInfo.poseX * poseThetaOffsetCosNew
                        + vInfo.poseYOffset;

        var poseXOffsetNew = newcurrentPoseX - recalculatedPoseX + vInfo.poseXOffset;
        var poseYOffsetNew = newcurrentPoseY - recalculatedPoseY + vInfo.poseYOffset;

        m_pastVisionUpdates.addLast(
                new VisionUpdate(
                        visUpd.visionPose,
                        visUpd.timestamp,
                        visUpd.translationVariance,
                        visUpd.rotationVariance,
                        new VisionUpdateOffset(
                                translationVarianceDetractorNew,
                                rotationVarianceDetractorNew,
                                poseXOffsetNew,
                                poseYOffsetNew,
                                poseThetaOffsetNew,
                                poseThetaOffsetCosNew,
                                poseThetaOffsetSinNew)));
    }

    private void moveVisionUpdatesToStack(VisionUpdate visionUpdate) {
        if (visionUpdate.timestamp <= m_pastVisionUpdates.getFirst().timestamp
                || visionUpdate.timestamp <= m_pastPosesRelative.getFirst().timestamp) {
            return;
        }

        while (m_pastVisionUpdates.getLast().timestamp > visionUpdate.timestamp) {
            m_unaddedVisionUpdates.add(m_pastVisionUpdates.removeLast());
        }

        m_unaddedVisionUpdates.add(visionUpdate);
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
