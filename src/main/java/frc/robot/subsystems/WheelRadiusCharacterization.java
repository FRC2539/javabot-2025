package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WheelRadiusCharacterization extends Command {
    private static final LoggedNetworkNumber characterizationSpeed =
      new LoggedNetworkNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
    private static final Pigeon2 pigeon = new Pigeon2(0,"");
    private double driveBaseRadius = 10;
    private static DoubleSupplier gyroYawRadSupplier = ()->((pigeon.getYaw().getValueAsDouble() * Math.PI) / 180);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

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

    private double[] startWheelPositions = new double[4];

    public WheelRadiusCharacterization(Direction omegaDirection, CommandSwerveDrivetrain drivetrain) {
        this.omegaDirection = omegaDirection;
        this.drive = drivetrain;

        addRequirements(drivetrain);
      }

    
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        for(int x = 0; x<4 ; x++)
        {
            startWheelPositions[x] = drive.getState().ModulePositions[0].distanceMeters;
        }



        omegaLimiter.reset(0);
    }
    

    public void execute()
    {
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadSupplier.getAsDouble();

        drive.driveRobotRelative(0, 0, omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositions = new double[4];
        for(int x = 0; x<4 ; x++)
        {
            wheelPositions[x] = drive.getState().ModulePositions[0].distanceMeters;
        }
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveBaseRadius) / averageWheelPosition;
    }

   
}