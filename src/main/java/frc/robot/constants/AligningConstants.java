package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AligningConstants {
    public static final double Kp = 0.9;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;

    public static final double leftOffset = -0.154; // sch -0.15 Hat -0.146 //home -0.152
    public static final double rightOffset = 0.188; // sch 0.188Hat 0.155 // home 0.165
    public static final double centerOffset = -0.226; // old center -0.222
    public static final double reefDistance =
            Units.inchesToMeters(33.25) / 2; // Units.feetToMeters(3) / 2;

    public static final double aligningAngleTolerance = Units.degreesToRadians(2);
    public static final double aligningXTolerance = Units.inchesToMeters(0.7);
    public static final double aligningYTolerance = Units.inchesToMeters(0.7);

    public static boolean robotInPlace(Pose2d robotPose, Pose2d targetPose, double offset) {
        Pose2d alignmentPose =
                targetPose.plus(
                        new Transform2d(
                                new Translation2d(AligningConstants.reefDistance, offset),
                                Rotation2d.k180deg));
        Pose2d offsetPose = robotPose.relativeTo(alignmentPose);
        return (offsetPose.getX() > -aligningXTolerance)
                && (Math.abs(offsetPose.getY()) < aligningYTolerance)
                && ((Math.abs(offsetPose.getRotation().getRadians()) % (2 * Math.PI))
                        < aligningAngleTolerance);
    }
}
