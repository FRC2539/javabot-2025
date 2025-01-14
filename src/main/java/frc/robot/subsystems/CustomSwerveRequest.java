package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class CustomSwerveRequest implements SwerveRequest{

    // private static class TempSwerveModule extends SwerveModule {
    //     public TempSwerveModule() {
    //         super(new SwerveModuleConstants(), "", 100000, 0);
    //     }
    // }

    private SwerveRequest userRequest;
    
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo).withDesaturateWheelSpeeds(false);

    private ChassisSpeeds desiredSpeedsForSlippingModule;

    private int indexOfSlippingWheel = -1;


    public void setIndexOfSlippingWheel(int index)
    {
        indexOfSlippingWheel = index;
    }

    public void setDesiredChassisSpeedsForSlippingModule(ChassisSpeeds desiredSpeeds)
    {
        desiredSpeedsForSlippingModule = desiredSpeeds;
    }

    public void setSwerveRequest(SwerveRequest swerveRequest)
    {
        userRequest = swerveRequest;
    }

    @Override

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... swerveModules)
    {

        SwerveModule[] modulesToApply = new SwerveModule[3];
        SwerveModule[] slippingModule = new SwerveModule[1];

    
        for(int i = 0; i < 4; i++)
        {
            if(i != indexOfSlippingWheel)
            {
                if(i > indexOfSlippingWheel)
                    modulesToApply[i - 1] = swerveModules[i];
                if(i < indexOfSlippingWheel)
                    modulesToApply[i] = swerveModules[i];    
            }
            else
            {
                slippingModule[0] = swerveModules[i];
            }
        }

        userRequest.apply(parameters, modulesToApply);

        return applyRobotSpeeds.withSpeeds(desiredSpeedsForSlippingModule).apply(parameters, slippingModule);

    }

}
