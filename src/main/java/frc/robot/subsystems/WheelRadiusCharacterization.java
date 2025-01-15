package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WheelRadiusCharacterization extends Command {
    private static final LoggedNetworkNumber characterizationSpeed =
            new LoggedNetworkNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 1);
    private double driveBaseRadius =
            Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY);
    private final DoubleSupplier gyroYawRadSupplier;

    private double lastGyroYawRads = 0.0;
    private double initialGyroYawRads = 0.0;

    public double currentEffectiveWheelRadius = 0.0;

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final double value;

        private Direction(double speed) {
            value = speed;
        }
    }

    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private final CommandSwerveDrivetrain drive;

    private final Timer startTimer = new Timer();

    private double[] startWheelPositions = new double[4];

    private boolean hasntStarted = true;

    public WheelRadiusCharacterization(
            Direction omegaDirection, CommandSwerveDrivetrain drivetrain) {
        this.omegaDirection = omegaDirection;
        this.drive = drivetrain;
        this.gyroYawRadSupplier = () -> drivetrain.getState().RawHeading.getRadians();

        addRequirements(drivetrain);
    }

    public void initialize() {
        // Reset
        hasntStarted = true;
        startTimer.restart();

        omegaLimiter.reset(0);
    }

    public void execute() {
        drive.driveRobotRelative(
                0, 0, omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));

        // Get yaw and wheel positions

        if (startTimer.hasElapsed(2) && hasntStarted) {
            initialGyroYawRads = gyroYawRadSupplier.getAsDouble();

            for (int x = 0; x < 4; x++) {
                startWheelPositions[x] =
                        drive.getState().ModulePositions[x].distanceMeters
                                / TunerConstants.BackLeft.WheelRadius;
            }

            hasntStarted = false;
        }

        if (hasntStarted) return;

        lastGyroYawRads = gyroYawRadSupplier.getAsDouble();

        double averageWheelPosition = 0.0;
        double[] wheelPositions = new double[4];
        for (int x = 0; x < 4; x++) {
            wheelPositions[x] =
                    drive.getState().ModulePositions[x].distanceMeters
                            / TunerConstants.BackLeft.WheelRadius;
        }
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius =
                ((lastGyroYawRads - initialGyroYawRads) * driveBaseRadius) / averageWheelPosition;

        Logger.recordOutput("/Drive/WheelRadiusCalculated", currentEffectiveWheelRadius * 100.0);
        Logger.recordOutput(
                "/Drive/WheelRadiusGyro", (lastGyroYawRads - initialGyroYawRads) * driveBaseRadius);
        Logger.recordOutput("/Drive/WheelPosition", averageWheelPosition);
    }
}
