package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    public static class visionIOInputs {
        public boolean connected = false;
        public int[] tagIDs = new int[0];
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public static record simpleTargetObservation(Rotation2d tx, Rotation2d ty) {};
    
    public static record PoseObservation(
        double timestamp,
        int tagCount,
        Pose3d pose,
        double ambiguity, 
        double averageTagDistance
    ) {}

    public default void updateINputs(visionIOInputs inputs) {}
}
