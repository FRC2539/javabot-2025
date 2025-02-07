package frc.robot.auto;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.GlobalConstants;
import java.util.ArrayList;
import java.util.List;

public class PathEval {
    private final RobotConfig configs;

    private final List<List<Translation2d>> keepOutZones;

    public PathEval() {
        try {
            configs = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException();
        }
        keepOutZones = new ArrayList<>();

        // Reef:
        keepOutZones.add(
                List.of(
                        new Translation2d(4.489, 5.004),
                        new Translation2d(3.617, 4.503),
                        new Translation2d(3.617, 3.536),
                        new Translation2d(4.489, 3.034),
                        new Translation2d(5.348, 3.536),
                        new Translation2d(5.348, 4.503)));

        // Station Right:
        keepOutZones.add(List.of(new Translation2d(0.025, 1.211), new Translation2d(1.697, 0)));

        // Station Left:
        keepOutZones.add(List.of(new Translation2d(0.025, 6.778), new Translation2d(1.697, 7.986)));
    }

    public Double timeOnly(PathPlannerPath path) {
        var idealTrajectory = path.getIdealTrajectory(configs);
        return -idealTrajectory.get().getTotalTimeSeconds();
    }

    public Double collisionsOnly(PathPlannerPath path) {
        var nextTraj = path.getIdealTrajectory(configs).get();
        double collisions = 0;
        for (var zone : keepOutZones) {
            for (PathPlannerTrajectoryState stateu : nextTraj.getStates()) {
                boolean collide = testRobotForCollision(stateu.pose, zone);
                if (!collide) {
                    collisions += stateu.timeSeconds;
                }
            }
        }
        return -collisions;
    }

    public Double timeAndCollisions(PathPlannerPath path) {
        return timeOnly(path) + collisionsOnly(path) * 30;
    }

    private static boolean keepOutTest(
            List<Translation2d> robotPoints, List<Translation2d> avoidPoints) {
        List<Pair<Translation2d, Translation2d>> pairs = new ArrayList<>();
        for (int i = 0; i < robotPoints.size() - 1; i++) {
            pairs.add(new Pair<>(robotPoints.get(i), robotPoints.get(i + 1)));
        }
        pairs.add(new Pair<>(robotPoints.get(robotPoints.size() - 1), robotPoints.get(0)));

        for (Pair<Translation2d, Translation2d> pair : pairs) {
            var a = pair.getFirst();
            var b = pair.getSecond();
            boolean isInside = false;
            for (Translation2d c : avoidPoints) {
                double crossProduct =
                        (b.getX() - a.getX()) * (c.getY() - a.getY())
                                - (b.getY() - a.getY()) * (c.getX() - a.getX());
                if (crossProduct > 0) {
                    isInside = true;
                    break;
                }
            }
            if (!isInside) {
                return true;
            }
        }
        return false;
    }

    private static boolean testRobotForCollision(Pose2d pose, List<Translation2d> avoidPoints) {
        List<Translation2d> robotPoints = new ArrayList<>();
        Translation2d[] bumperCorners =
                new Translation2d[] {
                    new Translation2d(
                            GlobalConstants.bumperLength / 2, GlobalConstants.bumperWidth / 2),
                    new Translation2d(
                            -GlobalConstants.bumperLength / 2, GlobalConstants.bumperWidth / 2),
                    new Translation2d(
                            -GlobalConstants.bumperLength / 2, -GlobalConstants.bumperWidth / 2),
                    new Translation2d(
                            GlobalConstants.bumperLength / 2, -GlobalConstants.bumperWidth / 2)
                };
        for (int i = 0; i < 4; i++) {
            robotPoints.add(
                    pose.plus(new Transform2d(bumperCorners[i], Rotation2d.kZero))
                            .getTranslation());
        }
        return keepOutTest(robotPoints, avoidPoints);
    }
}
