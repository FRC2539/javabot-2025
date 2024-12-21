package frc.robot.subsystems;

import com.pathplanner.lib.config.RobotConfig;
import frc.robot.constants.TunerConstants;

public class SwerveConstantsUtil {

    private static RobotConfig config;

    public static CommandSwerveDrivetrain getCommandSwerveDrivetrain() {
        return TunerConstants.createDrivetrain();
    }

    public static RobotConfig getRobotConfig() {
        if (config == null) {
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                // Handle exception as needed
                e.printStackTrace();
                throw new RuntimeException("Failed to load robot config from pathplanner.");
            }
        }
        return config;
    }
}
