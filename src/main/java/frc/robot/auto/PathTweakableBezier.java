package frc.robot.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.Random;

public class PathTweakableBezier implements Optimizer.Tweakable<PathTweakableBezier> {
    public static PathEval myEval = new PathEval();

    @FunctionalInterface
    public static interface Tweaker {
        public PathTweakableBezier tweak(PathTweakableBezier input, Random rand);
    }

    public final List<Pose2d> points;
    public final IdealStartingState startState;
    public final GoalEndState endState;
    public final Tweaker tweaker;

    /**
     * Creates a PathTweakableBezier path for optimization.
     *
     * @param startState The initial state of the path, including velocity and rotation
     * @param endState The goal end state of the path, including velocity and rotation
     * @param points List of Pose2d points defining the path waypoints
     */
    public PathTweakableBezier(
            IdealStartingState startState,
            GoalEndState endState,
            List<Pose2d> points,
            Tweaker tweaker) {
        this.startState = startState;
        this.endState = endState;
        this.points = points;
        this.tweaker = tweaker;
    }

    public PathTweakableBezier tweak(Random rand) {
        return tweaker.tweak(this, rand);
    }
    ;

    /**
     * @param pose
     * @param rand
     * @param step Step size to tweak by in meters.
     * @return
     */
    public static Pose2d tweakPoseTranslation(Pose2d pose, Random rand, double step) {
        return new Pose2d(
                pose.getX() + rand.nextDouble(-step, step),
                pose.getY() + rand.nextDouble(-step, step),
                pose.getRotation());
    }

    /**
     * @param pose
     * @param rand
     * @param step Step size to tweak by in radians.
     * @return
     */
    public static Pose2d tweakPoseRotation(Pose2d pose, Random rand, double step) {
        return new Pose2d(
                pose.getX(),
                pose.getY(),
                Rotation2d.fromRadians(
                        pose.getRotation().getRadians() + rand.nextDouble(-step, step)));
    }

    public double getReward() {
        return myEval.timeAndCollisions(getPath());
    }

    public PathPlannerPath getPath() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(points);

        PathPlannerPath myPath =
                new PathPlannerPath(
                        waypoints,
                        new PathConstraints(4.1, 7.0, Math.toRadians(540), Math.toRadians(720)),
                        startState,
                        endState);

        return myPath;
    }
}
