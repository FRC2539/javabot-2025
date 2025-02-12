package frc.lib.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class PinholeModel3D {
    public static Translation2d getTranslationToTarget(
            Translation3d targetDirectionFromCamera,
            Transform3d robotToCamera,
            double targetHeight) {
        double heightDifference = targetHeight - robotToCamera.getTranslation().getZ();

        Transform3d targetDirectionTransform =
                new Transform3d(targetDirectionFromCamera, new Rotation3d());

        double heightChangePerTranslation =
                new Transform3d(new Translation3d(), robotToCamera.getRotation())
                        .plus(targetDirectionTransform)
                        .getZ();

        double numberOfTransformsNeeded = heightDifference / heightChangePerTranslation;

        Transform3d finalPose =
                robotToCamera.plus(targetDirectionTransform.times(numberOfTransformsNeeded));

        return new Translation2d(finalPose.getX(), finalPose.getY());
    }
}
