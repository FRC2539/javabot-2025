package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerPathExtended;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Auto.DriveLocation;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.VisionConstants;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class AutoGenerators {
    private static RobotConfig configs;

    public static class AutoGeneratorRobot extends RobotBase {
        public RobotContainer container;

        public AutoGeneratorRobot() {
            container = new RobotContainer();

            try {
                configs = RobotConfig.fromGUISettings();
                DriveLocation[][] pairs =
                        new DriveLocation[][] {
                            {DriveLocation.SourceLeft, DriveLocation.A},
                            {DriveLocation.SourceLeft, DriveLocation.AB},
                            {DriveLocation.SourceLeft, DriveLocation.B},
                            {DriveLocation.SourceLeft, DriveLocation.C},
                            {DriveLocation.SourceLeft, DriveLocation.CD},
                            {DriveLocation.SourceLeft, DriveLocation.D},
                            {DriveLocation.SourceLeft, DriveLocation.G},
                            {DriveLocation.SourceLeft, DriveLocation.GH},
                            {DriveLocation.SourceLeft, DriveLocation.H},
                            {DriveLocation.SourceLeft, DriveLocation.I},
                            {DriveLocation.SourceLeft, DriveLocation.IJ},
                            {DriveLocation.SourceLeft, DriveLocation.J},
                            {DriveLocation.SourceLeft, DriveLocation.K},
                            {DriveLocation.SourceLeft, DriveLocation.KL},
                            {DriveLocation.SourceLeft, DriveLocation.L},
                            {DriveLocation.SourceRight, DriveLocation.A},
                            {DriveLocation.SourceRight, DriveLocation.AB},
                            {DriveLocation.SourceRight, DriveLocation.B},
                            {DriveLocation.SourceRight, DriveLocation.C},
                            {DriveLocation.SourceRight, DriveLocation.CD},
                            {DriveLocation.SourceRight, DriveLocation.D},
                            {DriveLocation.SourceRight, DriveLocation.E},
                            {DriveLocation.SourceRight, DriveLocation.EF},
                            {DriveLocation.SourceRight, DriveLocation.F},
                            {DriveLocation.SourceRight, DriveLocation.G},
                            {DriveLocation.SourceRight, DriveLocation.GH},
                            {DriveLocation.SourceRight, DriveLocation.H},
                            {DriveLocation.SourceRight, DriveLocation.K},
                            {DriveLocation.SourceRight, DriveLocation.KL},
                            {DriveLocation.SourceRight, DriveLocation.L},
                        };

                DriveLocation[] centers =
                        new DriveLocation[] {
                            DriveLocation.E,
                            DriveLocation.EF,
                            DriveLocation.F,
                            DriveLocation.G,
                            DriveLocation.GH,
                            DriveLocation.H,
                            DriveLocation.I,
                            DriveLocation.IJ,
                            DriveLocation.J,
                        };

                for (var value : pairs) {
                    PathPlannerPath yay = generateTrajectoryOffDriveLocation(value[0], value[1]);
                    PathPlannerTrajectory trajectory = yay.getIdealTrajectory(configs).get();
                    System.out.println(trajectory.getTotalTime().in(Seconds) + " seconds");
                    yay = optimizePath(yay, 1000, true);
                    trajectory = yay.getIdealTrajectory(configs).get();
                    System.out.println(trajectory.getTotalTime().in(Seconds) + " seconds");
                    PathPlannerPathExtended.toPathFile(yay, "Auto Generated");
                }

                for (var value : pairs) {
                    PathPlannerPath yay = generateTrajectoryDriveLocation(value[1], value[0]);
                    PathPlannerTrajectory trajectory = yay.getIdealTrajectory(configs).get();
                    System.out.println(trajectory.getTotalTime().in(Seconds) + " seconds");
                    yay = optimizePath(yay, 1000);
                    trajectory = yay.getIdealTrajectory(configs).get();
                    System.out.println(trajectory.getTotalTime().in(Seconds) + " seconds");
                    PathPlannerPathExtended.toPathFile(yay, "Auto Generated");
                }

                var startingPoses = new ArrayList<Pair<Pose2d, String>>();
                {
                    startingPoses.add(
                            new Pair<>(
                                    new Pose2d(7.139, 0.521, Rotation2d.fromDegrees(180)),
                                    "RightStart"));
                    startingPoses.add(
                            new Pair<>(
                                    new Pose2d(7.139, 7.553, Rotation2d.fromDegrees(180)),
                                    "LeftStart"));
                    startingPoses.add(
                            new Pair<>(
                                    new Pose2d(
                                            7.139,
                                            (0.521 + 7.553) / 2,
                                            Rotation2d.fromDegrees(180)),
                                    "CenterStart"));
                    startingPoses.add(
                            new Pair<>(
                                    new Pose2d(
                                            7.139,
                                            ((0.521 + 7.553) / 2 + 0.521) / 2,
                                            Rotation2d.fromDegrees(180)),
                                    "CenterRightStart"));
                    startingPoses.add(
                            new Pair<>(
                                    new Pose2d(
                                            7.139,
                                            ((0.521 + 7.553) / 2 + 7.553) / 2,
                                            Rotation2d.fromDegrees(180)),
                                    "CenterLeftStart"));
                }

                for (var startingPose : startingPoses) {

                    for (var value : centers) {
                        Pose2d endPose =
                                VisionConstants.aprilTagLayout
                                        .getTagPose(value.tagBlue)
                                        .get()
                                        .toPose2d()
                                        .plus(
                                                new Transform2d(
                                                        new Translation2d(
                                                                Units.feetToMeters(3) / 2 + 0.3,
                                                                value.offset),
                                                        Rotation2d.k180deg));
                        PathPlannerPath yay =
                                generateTrajectory(startingPose.getFirst(), endPose, 1);
                        yay.name = startingPose.getSecond() + "-" + value.name();
                        PathPlannerTrajectory trajectory = yay.getIdealTrajectory(configs).get();
                        System.out.println(trajectory.getTotalTime().in(Seconds) + " seconds");
                        yay = optimizePath(yay, 1000, true);
                        trajectory = yay.getIdealTrajectory(configs).get();
                        System.out.println(trajectory.getTotalTime().in(Seconds) + " seconds");
                        PathPlannerPathExtended.toPathFile(yay, "Auto Generated");
                    }
                }
                // List<Translation2d> avoidCorners = List.of(
                //     new Translation2d(4.489, 5.004),
                //     new Translation2d(3.617, 4.503),
                //     new Translation2d(3.617, 3.536),
                //     new Translation2d(4.489, 3.034),
                //     new Translation2d(5.348, 3.536),
                //     new Translation2d(5.348, 4.503)
                // );

                // System.out.println(testRobotForCollision(new Pose2d(4.694,3.853,
                // Rotation2d.fromDegrees(0)), avoidCorners));
            } catch (Exception e) {
                throw new RuntimeException("Failed to generate trajectory.");
            }
        }

        public void startCompetition() {
            suppressExitWarning(true);
        }

        public void endCompetition() {}
    }

    public static void main(String... args) throws IOException, InterruptedException {
        // Delete existing auto-generated paths
        System.out.println("Deleting existing auto-generated paths...");
        java.io.File deployDir = new java.io.File("src/main/deploy/pathplanner/paths");
        if (deployDir.exists()) {
            for (java.io.File file : deployDir.listFiles()) {
                if (file.getName().endsWith(".path")) {
                    try {
                        String content =
                                new String(java.nio.file.Files.readAllBytes(file.toPath()));
                        if (content.contains("\"folder\": \"Auto Generated\"")) {
                            file.delete();
                        }
                    } catch (IOException e) {
                        System.err.println("Failed to read/delete file: " + file.getName());
                    }
                }
            }
        }
        System.out.println("Finished deleting existing auto-generated paths.");

        boolean isWindows = System.getProperty("os.name").startsWith("Windows");
        var gradleBuilder =
                new ProcessBuilder(
                        isWindows ? "gradlew.bat" : "./gradlew",
                        "simulateJava",
                        "-x",
                        "test",
                        "-x",
                        "spotlessApply",
                        "-x",
                        "spotlessCheck");
        gradleBuilder.environment().put("GENERATE_TRAJECTORIES", "true");

        // Redirect error stream to output stream
        gradleBuilder.redirectErrorStream(true);

        var gradle = gradleBuilder.start();
        System.out.println("Started gradle subtask.");

        // Read the output stream in real-time
        try (var reader =
                new java.io.BufferedReader(
                        new java.io.InputStreamReader(gradle.getInputStream()))) {
            String line;
            while ((line = reader.readLine()) != null) {
                System.out.println(line);
            }
        }

        gradle.waitFor();
        System.out.println("Finished gradle subtask.");
    }

    private static PathPlannerPath generateTrajectory(
            Pose2d startPose, Pose2d endPose, double endVel, Pose2d... inMiddlePoses) {
        List<PathPoint> points = new ArrayList<>();

        // Create waypoints for path
        List<Waypoint> waypoints = new ArrayList<Waypoint>();

        waypoints.add(
                new Waypoint(
                        null,
                        startPose.getTranslation(),
                        startPose.plus(new Transform2d(-2, 0, Rotation2d.kZero)).getTranslation()));
        for (Pose2d pose : inMiddlePoses) {
            waypoints.add(
                    new Waypoint(
                            startPose.getTranslation().interpolate(pose.getTranslation(), 0.8),
                            pose.getTranslation(),
                            endPose.getTranslation().interpolate(pose.getTranslation(), 0.8)));
        }
        waypoints.add(
                new Waypoint(
                        endPose.plus(new Transform2d(-2, 0, Rotation2d.kZero)).getTranslation(),
                        endPose.getTranslation(),
                        null));
        // Create path with constraints
        PathConstraints constraints =
                new PathConstraints(
                        4.1, // Max velocity (m/s)
                        7.0, // Max acceleration (m/s^2)
                        540.0, // Max angular velocity (deg/s)
                        720.0 // Max angular acceleration (deg/s^2)
                        );

        IdealStartingState startState =
                new IdealStartingState(
                        0.0, // Starting velocity
                        startPose.getRotation() // Starting rotation
                        );

        GoalEndState endState =
                new GoalEndState(
                        endVel, // Goal velocity
                        endPose.getRotation() // Goal rotation
                        );

        var path = new PathPlannerPath(waypoints, constraints, startState, endState);
        return path;
    }

    private static PathPlannerPath generateTrajectoryDriveLocation(
            DriveLocation start, DriveLocation end) {
        // Calculate start and end poses using April tag positions
        Pose2d startPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(start.tagBlue)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(Units.feetToMeters(3) / 2, start.offset),
                                        Rotation2d.k180deg));

        Pose2d endPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(end.tagBlue)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(Units.feetToMeters(3) / 2, end.offset),
                                        Rotation2d.k180deg));

        var output = generateTrajectory(startPose, endPose, 0);
        output.name = start.name() + "-" + end.name();

        return output;
    }

    private static PathPlannerPath generateTrajectoryOffDriveLocation(
            DriveLocation start, DriveLocation end) {
        // Calculate start and end poses using April tag positions
        Pose2d startPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(start.tagBlue)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(Units.feetToMeters(3) / 2, start.offset),
                                        Rotation2d.k180deg));

        Pose2d endPose =
                VisionConstants.aprilTagLayout
                        .getTagPose(end.tagBlue)
                        .get()
                        .toPose2d()
                        .plus(
                                new Transform2d(
                                        new Translation2d(
                                                Units.feetToMeters(3) / 2 + 0.3, end.offset),
                                        Rotation2d.k180deg));

        var output = generateTrajectory(startPose, endPose, 1);
        output.name = start.name() + "-" + end.name();

        return output;
    }

    // points must come in in a counterclockwise order
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

    private static PathPlannerPath optimizePath(PathPlannerPath path, int iterations) {
        return optimizePath(path, iterations, false);
    }

    private static PathPlannerPath optimizePath(
            PathPlannerPath path, int iterations, boolean perserveEndingDirection) {
        Random rand = new Random();
        rand.setSeed(13292853230L);

        List<Translation2d> avoidCorners =
                List.of(
                        new Translation2d(4.489, 5.004),
                        new Translation2d(3.617, 4.503),
                        new Translation2d(3.617, 3.536),
                        new Translation2d(4.489, 3.034),
                        new Translation2d(5.348, 3.536),
                        new Translation2d(5.348, 4.503));
        List<Translation2d> avoidCornersB =
                List.of(new Translation2d(0.025, 1.211), new Translation2d(1.697, 0));
        List<Translation2d> avoidCornersC =
                List.of(new Translation2d(0.025, 6.778), new Translation2d(1.697, 7.986));

        PathPlannerPath bestPath = path;
        double bestTime = Double.POSITIVE_INFINITY;
        PathPlannerPath absoluteBestPath = bestPath;
        double absoluteBestTime = bestTime;
        for (int i = 0; i < iterations; i++) {
            var nextPath = monteCarloStep(bestPath, 0.1, perserveEndingDirection, rand);
            PathPlannerTrajectory nextTraj = nextPath.getIdealTrajectory(configs).get();
            double collisions = 0;
            for (PathPlannerTrajectoryState stateu : nextTraj.getStates()) {
                boolean collide = testRobotForCollision(stateu.pose, avoidCorners);
                if (!collide) {
                    collisions += stateu.timeSeconds;
                }
                collide = testRobotForCollision(stateu.pose, avoidCornersB);
                if (!collide) {
                    collisions += stateu.timeSeconds;
                }
                collide = testRobotForCollision(stateu.pose, avoidCornersC);
                if (!collide) {
                    collisions += stateu.timeSeconds;
                }
            }
            double nextTime =
                    nextPath.getIdealTrajectory(configs).get().getTotalTimeSeconds()
                            + collisions * 100;
            if (nextTime < bestTime || rand.nextDouble() > 0.8) {
                bestPath = nextPath;
                bestTime = nextTime;
            }
            if (nextTime < absoluteBestTime) {
                absoluteBestPath = nextPath;
                absoluteBestTime = nextTime;
            }
        }
        absoluteBestPath.name = path.name;
        return absoluteBestPath;
    }

    private static PathPlannerPath monteCarloStep(
            PathPlannerPath path, double size, boolean perserveEndingDirection, Random random) {
        var finalWaypoints = new ArrayList<>(path.getWaypoints());
        for (int i = 0; i < finalWaypoints.size(); i++) {
            var waypoint = finalWaypoints.get(i);
            var anchor = waypoint.anchor();
            var lastControl = waypoint.prevControl();
            var nextControl = waypoint.nextControl();

            if (i != 0 && i != finalWaypoints.size() - 1) {
                Translation2d randomTranslation =
                        new Translation2d(
                                (random.nextDouble() - 0.5) * size * 5,
                                (random.nextDouble() - 0.5) * size * 5);
                anchor = anchor.plus(randomTranslation);
                lastControl = lastControl.plus(randomTranslation);
                nextControl = nextControl.plus(randomTranslation);
            }

            Rotation2d rotationAmount =
                    Rotation2d.fromRotations((random.nextDouble() - 0.5) * size);
            double lastStreach = (random.nextDouble() - 0.5) * size + 1;
            double nextStreach = (random.nextDouble() - 0.5) * size + 1;
            if (lastControl != null) {
                if (!(perserveEndingDirection && i == finalWaypoints.size() - 1))
                    lastControl = lastControl.rotateAround(anchor, rotationAmount);
                lastControl = lastControl.minus(anchor).times(lastStreach).plus(anchor);
            }
            if (nextControl != null) {
                if (!(perserveEndingDirection && i == finalWaypoints.size() - 1))
                    nextControl = nextControl.rotateAround(anchor, rotationAmount);
                nextControl = nextControl.minus(anchor).times(nextStreach).plus(anchor);
            }

            var nextWaypoint = new Waypoint(lastControl, anchor, nextControl);
            finalWaypoints.set(i, nextWaypoint);
        }

        return new PathPlannerPath(
                finalWaypoints,
                path.getGlobalConstraints(),
                path.getIdealStartingState(),
                path.getGoalEndState());
    }
}
