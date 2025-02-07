package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerPathExtended;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Auto.DriveLocation;
import frc.robot.auto.Optimizer;
import frc.robot.auto.PathEval;
import frc.robot.auto.PathTweakable;
import frc.robot.constants.VisionConstants;
import java.io.IOException;
import java.util.ArrayList;

public class AutoGenerators {

    public static class AutoGeneratorRobot extends RobotBase {
        public RobotContainer container;

        public AutoGeneratorRobot() {
            container = new RobotContainer();

            try {
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

                var evaler = new PathEval();

                for (var value : pairs) {
                    var start = getPoseFromTag(value[0]);
                    var end = getPoseFromTag(value[1])
                    .plus(new Transform2d(-0.3, 0, Rotation2d.kZero));

                    PathTweakable yay =
                            new PathTweakable(
                                    start,
                                    start.interpolate(end, 0.3).getTranslation(),
                                    end.plus(new Transform2d(-1, 0, Rotation2d.kZero)).getTranslation(),
                                    end);
                    yay.setEndVelocity(1);
                    yay.setEndAngleTweak(false);
                    yay.setEvalFunction(evaler::timeAndCollisions);
                    yay = Optimizer.optimize(yay);
                    PathPlannerPath optimized = yay.getPath();
                    // PathPlannerTrajectory trajectory =
                    // optimized.getIdealTrajectory(configs).get();
                    // System.out.println(trajectory.getTotalTime().in(Seconds) + " seconds");
                    optimized.name = value[0].name() + "-" + value[1].name();
                    PathPlannerPathExtended.toPathFile(optimized, "Auto Generated");
                }

                for (var value : pairs) {

                    PathTweakable yay =
                            new PathTweakable(getPoseFromTag(value[1]), getPoseFromTag(value[0]));
                    yay.setEvalFunction(evaler::timeAndCollisions);
                    yay = Optimizer.optimize(yay);
                    PathPlannerPath optimized = yay.getPath();
                    optimized.name = value[1].name() + "-" + value[0].name();
                    PathPlannerPathExtended.toPathFile(optimized, "Auto Generated");
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
                    var start = startingPose.getFirst();
                    
                    for (var value : centers) {
                        var end = getPoseFromTag(value)
                            .plus(new Transform2d(-0.3, 0, Rotation2d.kZero));
                        PathTweakable yay =
                                new PathTweakable(
                                        start,
                                        start.interpolate(end, 0.3).getTranslation(),
                                        end.plus(new Transform2d(-1, 0, Rotation2d.kZero)).getTranslation(),
                                        end);
                        yay.setEndVelocity(1);
                        yay.setEndAngleTweak(false);
                        yay.setEvalFunction(evaler::timeAndCollisions);
                        yay = Optimizer.optimize(yay);
                        PathPlannerPath optimized = yay.getPath();

                        optimized.name = startingPose.getSecond() + "-" + value.name();
                        PathPlannerPathExtended.toPathFile(optimized, "Auto Generated");
                    }
                }
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

    private static Pose2d getPoseFromTag(DriveLocation location) {
        return VisionConstants.aprilTagLayout
                .getTagPose(location.tagBlue)
                .get()
                .toPose2d()
                .plus(
                        new Transform2d(
                                new Translation2d(Units.feetToMeters(3) / 2, location.offset),
                                Rotation2d.k180deg));
    }
}
