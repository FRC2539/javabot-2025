package frc.robot.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Random;
import java.util.function.Function;

public class PathTweakable implements Optimizer.Tweakable<PathTweakable> {
    private Pose2d startPose;
    private Translation2d startControl;
    private Translation2d endControl;
    private Pose2d endPose;
    private double endVelocity;
    private double startVelocity;
    private Function<PathPlannerPath, Double> evalFunction;

    private boolean allowAngleTweakStart = true;
    private boolean allowAngleTweakEnd = true;

    private PathTweakable() {}

    public void setEvalFunction(Function<PathPlannerPath, Double> evalFunction) {
        this.evalFunction = evalFunction;
    }

    public void setEndVelocity(double velocity) {
        endVelocity = velocity;
    }

    public void setStartVelocity(double velocity) {
        startVelocity = velocity;
    }

    public PathTweakable(Pose2d startPose, Pose2d endPose) {
        this();
        this.startPose = startPose;
        this.endPose = endPose;
        startControl = startPose.interpolate(endPose, 0.3).getTranslation();
        endControl = startPose.interpolate(endPose, 0.7).getTranslation();
    }

    public PathTweakable(Pose2d startPose, Translation2d startControl, Translation2d endControl, Pose2d endPose) {
        this();
        this.startPose = startPose;
        this.endPose = endPose;
        this.startControl = startControl;
        this.endControl = endControl;
    }

    public void setEndAngleTweak(boolean allow) {
        allowAngleTweakEnd = allow;
    }

    public void setStartAngleTweak(boolean allow) {
        allowAngleTweakStart = allow;
    }

    public PathTweakable tweak(Random rand) {
        var nextStartControl = startControl;
        var nextEndControl = endControl;

        if (allowAngleTweakStart) {
            nextStartControl =
                    nextStartControl.rotateAround(
                            startPose.getTranslation(),
                            Rotation2d.fromRotations(rand.nextDouble() * 0.1));
        }

        if (allowAngleTweakEnd) {
            nextEndControl =
                    nextEndControl.rotateAround(
                            endPose.getTranslation(),
                            Rotation2d.fromRotations(rand.nextDouble() * 0.1));
        }

        nextStartControl =
                nextStartControl
                        .minus(startPose.getTranslation())
                        .times(1 + (rand.nextDouble() - 0.5) * 0.1)
                        .plus(startPose.getTranslation());
        nextEndControl =
                nextEndControl
                        .minus(endPose.getTranslation())
                        .times(1 + (rand.nextDouble() - 0.5) * 0.1)
                        .plus(endPose.getTranslation());

        var nextTweakable = new PathTweakable();
        nextTweakable.startPose = startPose;
        nextTweakable.endPose = endPose;
        nextTweakable.startControl = nextStartControl;
        nextTweakable.endControl = nextEndControl;
        nextTweakable.endVelocity = endVelocity;
        nextTweakable.startVelocity = startVelocity;
        nextTweakable.evalFunction = evalFunction;
        nextTweakable.allowAngleTweakEnd = allowAngleTweakEnd;
        nextTweakable.allowAngleTweakStart = allowAngleTweakStart;
        return nextTweakable;
    }

    public double getReward() {

        return evalFunction.apply(getPath());
    }

    public PathPlannerPath getPath() {
        var myWaypoints = new ArrayList<Waypoint>();
        myWaypoints.add(new Waypoint(null, startPose.getTranslation(), startControl));
        myWaypoints.add(new Waypoint(endControl, endPose.getTranslation(), null));

        var myPath =
                new PathPlannerPath(
                        myWaypoints,
                        new PathConstraints(4.1, 7.0, Math.toRadians(540), Math.toRadians(720)),
                        new IdealStartingState(startVelocity, startPose.getRotation()),
                        new GoalEndState(endVelocity, endPose.getRotation()));

        return myPath;
    }
}
