package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class AligningConstants {
    public static AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static Translation2d[] scoringOffset =
            new Translation2d[] {
                new Translation2d(0, 0),
                new Translation2d(0, 0),
                new Translation2d(0, 0),
                new Translation2d(0, 0),
                new Translation2d(0, 0),
                new Translation2d(0, 0)
            };

    public static double targetTxL = 0.0;
    public static double targetTxR = 0.0;
    public static double angleDeadband = 3.0;
}
