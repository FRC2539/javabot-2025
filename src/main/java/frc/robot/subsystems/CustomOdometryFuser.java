package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.CircularBuffer;
import java.util.Comparator;
import java.util.Optional;
import java.util.Queue;
import java.util.Stack;
import java.util.concurrent.PriorityBlockingQueue;

/** This is a custom odometry fuser. It is thread safe in theory. (Matthew approved.) */
public class CustomOdometryFuser {
    private final boolean kUtilizeTurningCorrection = false;

    private final Object readingLock = new Object();

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

    private VisionUpdate m_lastVisionUpdate;

    private final int m_pastVisionUpdatesMaxSize = 10;
    private CircularBuffer<VisionUpdate> m_pastVisionUpdates =
            new CircularBuffer<>(m_pastVisionUpdatesMaxSize);

    private Stack<VisionUpdate> m_unaddedVisionUpdates = new Stack<>();

    private Queue<VisionUpdate> m_unaddedVisionUpdatesExternal =
            new PriorityBlockingQueue<>(
                    11,
                    new Comparator<VisionUpdate>() {
                        public int compare(VisionUpdate a, VisionUpdate b) {
                            return Double.compare(a.timestamp, b.timestamp);
                        }
                    });

    public CustomOdometryFuser() {
        m_pastVisionUpdates.addLast(
                new VisionUpdate(Pose2d.kZero, 0, 0, 0, new VisionUpdateOffset(0, 0, 0, 0, 0)));
        m_lastVisionUpdate = m_pastVisionUpdates.getLast();
        m_pastPosesRelative.addLast(new TimedInfo(0, 0, 0, 0, 0, 0));
    }

    public void resetPose(Pose2d newPose, double timestamp) {
        addVisionUpdate(newPose, timestamp, 0.0, 0.0);
    }

    public Pose2d getPose() {
        final TimedInfo tInfo;
        final VisionUpdateOffset vInfo;
        synchronized (readingLock) {
            tInfo = m_pastPosesRelative.getLast();
            vInfo = m_lastVisionUpdate.visionUpdateOffset;
        }

        double currentPoseX =
                tInfo.poseX * vInfo.poseThetaOffsetCos
                        - tInfo.poseY * vInfo.poseThetaOffsetSin
                        + vInfo.poseXOffset;
        double currentPoseY =
                tInfo.poseY * vInfo.poseThetaOffsetCos
                        + tInfo.poseX * vInfo.poseThetaOffsetSin
                        + vInfo.poseYOffset;
        double currentPoseTheta = tInfo.poseTheta + vInfo.poseThetaOffset;

        return new Pose2d(currentPoseX, currentPoseY, new Rotation2d(currentPoseTheta));
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        final TimedInfo tInfo;
        final VisionUpdateOffset vInfo;

        synchronized (readingLock) {
            int timestampIndex = getTimestampIndex(timestamp);

            if (timestampIndex == -1) {
                return Optional.empty();
            }

            tInfo = m_pastPosesRelative.get(timestampIndex);
            vInfo = m_lastVisionUpdate.visionUpdateOffset;
        }

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
        return m_pastPosesRelative.getLast().xyVariance
                + m_lastVisionUpdate.visionUpdateOffset.translationVarianceDetractor;
    }

    public double getPhysicalPoseStdDev() {
        return Math.sqrt(getPhysicalPoseVariance());
    }

    public double getRotationalPoseVariance() {
        return m_pastPosesRelative.getLast().thetaVariance
                + m_lastVisionUpdate.visionUpdateOffset.rotationVarianceDetractor;
    }

    public double getRotationalPoseStdDev() {
        return Math.sqrt(getRotationalPoseVariance());
    }

    // You can use this to add a new measurement to the odometry fuser using this. This also
    // processes all the vision updates.
    public void addSwerveMeasurementTwist(
            Twist2d twist, double timestamp, double translationVariance, double rotationVariance) {
        synchronized (this) {
            final Pose2d postExp = Pose2d.kZero.exp(twist);

            double pdx = postExp.getX();
            double pdy = postExp.getY();
            double dtheta = postExp.getRotation().getRadians();

            final var tInfo = m_pastPosesRelative.getLast();

            double dx = Math.cos(tInfo.poseTheta) * pdx - Math.sin(tInfo.poseTheta) * pdy;
            double dy = Math.cos(tInfo.poseTheta) * pdy + Math.sin(tInfo.poseTheta) * pdx;

            final double dt = timestamp - tInfo.timestamp;

            // This is technically not correctly considered right now.
            if (kUtilizeTurningCorrection) {
                double nextTheta = tInfo.thetaVariance + rotationVariance * dt;
                double lengthMod = Math.exp(-0.5 * nextTheta);
                dx *= lengthMod;
                dy *= lengthMod;
                double lengthVarianceAddition = (1 - Math.exp(-nextTheta)) * (dx * dx + dy * dy);
                translationVariance += lengthVarianceAddition;
            }

            synchronized (readingLock) {
                m_pastPosesRelative.addLast(
                        new TimedInfo(
                                tInfo.poseX + dx,
                                tInfo.poseY + dy,
                                tInfo.poseTheta + dtheta,
                                tInfo.xyVariance + translationVariance * dt,
                                tInfo.thetaVariance + rotationVariance * dt,
                                timestamp));
            }

            while (true) {
                VisionUpdate lastVisionUpdate = m_unaddedVisionUpdatesExternal.poll();
                if (lastVisionUpdate == null) {
                    break;
                }
                moveVisionUpdateOffOuterStack(lastVisionUpdate);
            }

            synchronized (readingLock) {
                m_lastVisionUpdate = m_pastVisionUpdates.getLast();
            }
        }
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
    public void addVisionUpdate(
            Pose2d visionPose,
            double timestamp,
            double translationVariance,
            double rotationVariance) {

        m_unaddedVisionUpdatesExternal.add(
                new VisionUpdate(
                        visionPose, timestamp, translationVariance, rotationVariance, null));
    }

    private void moveVisionUpdateOffOuterStack(VisionUpdate visionUpdate) {
        moveVisionUpdatesToStack(visionUpdate);

        while (!m_unaddedVisionUpdates.isEmpty()) {
            moveVisionUpdateOffStack();
        }
    }

    private void moveVisionUpdateOffStack() {
        final var visUpd = m_unaddedVisionUpdates.pop();

        final int timestampIndex = getTimestampIndex(visUpd.timestamp);

        final double visionX = visUpd.visionPose.getX();
        final double visionY = visUpd.visionPose.getY();
        final double visionTheta = visUpd.visionPose.getRotation().getRadians();

        final var tInfo = m_pastPosesRelative.get(timestampIndex);
        final var vInfo = m_pastVisionUpdates.getLast().visionUpdateOffset;

        final double currentPoseX =
                tInfo.poseX * vInfo.poseThetaOffsetCos
                        - tInfo.poseY * vInfo.poseThetaOffsetSin
                        + vInfo.poseXOffset;
        final double currentPoseY =
                tInfo.poseY * vInfo.poseThetaOffsetSin
                        + tInfo.poseX * vInfo.poseThetaOffsetCos
                        + vInfo.poseYOffset;
        final double currentPoseTheta = tInfo.poseTheta + vInfo.poseThetaOffset;

        final double physicalPoseVariance = tInfo.xyVariance + vInfo.translationVarianceDetractor;
        final double rotationalPoseVariance = tInfo.thetaVariance + vInfo.rotationVarianceDetractor;

        final double newcurrentPoseX =
                squareMerge(
                        currentPoseX, physicalPoseVariance, visionX, visUpd.translationVariance);
        final double newcurrentPoseY =
                squareMerge(
                        currentPoseY, physicalPoseVariance, visionY, visUpd.translationVariance);
        final double newcurrentPoseTheta =
                squareMerge(
                        currentPoseTheta,
                        rotationalPoseVariance,
                        visionTheta,
                        visUpd.rotationVariance);

        final double newPhysicalPoseVariance = tMerge(physicalPoseVariance, visUpd.translationVariance);
        final double newRotationalPoseVariance = tMerge(rotationalPoseVariance, visUpd.rotationVariance);

        final double translationVarianceDetractorNew =
                newPhysicalPoseVariance - physicalPoseVariance + vInfo.translationVarianceDetractor;
        final double rotationVarianceDetractorNew =
                newRotationalPoseVariance
                        - rotationalPoseVariance
                        + vInfo.rotationVarianceDetractor;

        final double poseThetaOffsetNew = newcurrentPoseTheta - currentPoseTheta + vInfo.poseThetaOffset;
        final double poseThetaOffsetCosNew = Math.cos(poseThetaOffsetNew);
        final double poseThetaOffsetSinNew = Math.sin(poseThetaOffsetNew);

        final double recalculatedPoseX =
                tInfo.poseX * poseThetaOffsetCosNew
                        - tInfo.poseY * poseThetaOffsetSinNew
                        + vInfo.poseXOffset;
        final double recalculatedPoseY =
                tInfo.poseY * poseThetaOffsetSinNew
                        + tInfo.poseX * poseThetaOffsetCosNew
                        + vInfo.poseYOffset;

        final var poseXOffsetNew = newcurrentPoseX - recalculatedPoseX + vInfo.poseXOffset;
        final var poseYOffsetNew = newcurrentPoseY - recalculatedPoseY + vInfo.poseYOffset;

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
        final double a2 = 1 / (a_var);
        final double b2 = 1 / (b_var);
        final double denom = 1 / (a2 + b2);

        return (a * a2 + b * b2) / denom;
    }

    private double tMerge(double a_var, double b_var) {
        return 1 / (1 / a_var + 1 / b_var);
    }
}
