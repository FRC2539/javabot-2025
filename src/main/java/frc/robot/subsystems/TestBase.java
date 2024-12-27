package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TestBase extends SubsystemBase{
    LoggedNetworkNumber motorOneSpeed = new LoggedNetworkNumber("motorOne", 1);
    LoggedNetworkNumber motorTwoSpeed = new LoggedNetworkNumber("motorTwo", 1);
    LoggedNetworkNumber motorThreeSpeed = new LoggedNetworkNumber("motorThree", 1);
    DigitalInput sensor = new DigitalInput(6);
    
    LoggedNetworkBoolean simSensor = new LoggedNetworkBoolean("simSensor", false);
    

    

    private BooleanSupplier getSensorValue(){
        if (Robot.isSimulation()) {
            BooleanSupplier simAnswer = simSensor::get;
            return simAnswer;
        }
        BooleanSupplier realAnswer = sensor::get;
        return realAnswer;
    }


    

    TalonFX motorOne = new TalonFX(9);
    TalonFX motorTwo = new TalonFX(10);
    TalonFX motorThree = new TalonFX(11);
    TalonFX motorOneFollower = new TalonFX(19);
    TalonFX motorTwoFollower = new TalonFX(20);
    TalonFX motorThreeFollower = new TalonFX(21);

    private void setSpeedAmount(int id, double speed){
        if(id==1){
            motorOne.set(speed);
        }
        else if(id==2){
            motorTwo.set(speed);
        }
        else{
            motorThree.set(speed);
        }
    }

    

    public Command run() {
        return run(() -> {
            setSpeedAmount(1, motorOneSpeed.get());
            setSpeedAmount(2,motorTwoSpeed.get());
            setSpeedAmount(3, motorThreeSpeed.get());
        
        })
        .beforeStarting(() -> {
            Logger.recordOutput("Drive/driving", "running");
        }).until(getSensorValue());
        
    }

    public Command followerRun() {
        return run(() -> {
            motorOneFollower.set(motorOne.get());
            motorTwoFollower.set(motorTwo.get());
            motorThreeFollower.set(motorThree.get());
        }).beforeStarting(() -> {
            Logger.recordOutput("Drive/follower", "follower enabled");
        });
    }

    public Command noSpeed() {
        return run(() -> {
            setSpeedAmount(1, 0);
            setSpeedAmount(2,0);
            setSpeedAmount(3, 0);
        }).beforeStarting(() -> {
            Logger.recordOutput("Drive/driving", "idle");
        });

    }

    public Command upTheSpeed() {
        return run(() -> {
            motorOneSpeed.set(motorOneSpeed.get() + 0.005); 
            motorTwoSpeed.set(motorTwoSpeed.get() + 0.005); 
            motorThreeSpeed.set(motorThreeSpeed.get() + 0.005); 
        }).beforeStarting(() -> {
            Logger.recordOutput("Drive/setspeed", "upped speed");
        });
    }

    public Command downTheSpeed() {
        return run(() -> {
            motorOneSpeed.set(motorOneSpeed.get() - 0.005); 
            motorTwoSpeed.set(motorTwoSpeed.get() - 0.005); 
            motorThreeSpeed.set(motorThreeSpeed.get() - 0.005); 
        }).beforeStarting(() -> {
            Logger.recordOutput("Drive/setspeed", "downed speed");
        });
    }

   @Override
   public void periodic(){
    if(motorOneSpeed.get() > 1){
        motorOneSpeed.set(1);
    }
    if(motorTwoSpeed.get() > 1){
        motorTwoSpeed.set(1);
    }
    if(motorThreeSpeed.get() > 1){
        motorThreeSpeed.set(1);
    }
    
    if(motorOneSpeed.get() < 0){
        motorOneSpeed.set(0);
    }
    if(motorTwoSpeed.get() < 0){
        motorTwoSpeed.set(0);
    }
    if(motorThreeSpeed.get() < 0){
        motorThreeSpeed.set(0);
    }
    Logger.recordOutput("Drive/motor one speed", motorOneSpeed.get());
   }

    
}

