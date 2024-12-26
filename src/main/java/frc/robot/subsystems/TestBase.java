package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
    DigitalInput ongodrizzler = new DigitalInput(6);
    Boolean the = ongodrizzler.get();
    LoggedNetworkBoolean sigGod = new LoggedNetworkBoolean("sigGod", false);
    Boolean f = sigGod.get();

    

    private BooleanSupplier getTheRizzFromTheRizzler(){
        if (Robot.isSimulation()) {
            BooleanSupplier simAnswer = sigGod::get;
            return simAnswer;
        }
        BooleanSupplier realAnswer = ongodrizzler::get;
        return realAnswer;
    }

    TalonFX motorOne = new TalonFX(9);
    TalonFX motorTwo = new TalonFX(10);
    TalonFX motorThree = new TalonFX(11);

    private void setSigmaSpeed(int eyedee, double spid){
        if(eyedee==1){
            motorOne.set(spid);
        }
        else if(eyedee==2){
            motorTwo.set(spid);
        }
        else{
            motorThree.set(spid);
        }
    }

    public Command skibidiBazooka(){
        return run(() -> setSigmaSpeed(1,0.999));

    }

    // public Command runWhile(){
        
    //     return runReallyReallyReallyReallyReallyReallyReallyFast().until(getTheRizzFromTheRizzler());
    // }

    public Command runReallyReallyReallyReallyReallyReallyReallyFast() {
        return run(() -> {
            setSigmaSpeed(1, motorOneSpeed.get());
            setSigmaSpeed(2,motorTwoSpeed.get());
            setSigmaSpeed(3, motorThreeSpeed.get());
        
        })
        .beforeStarting(() -> {
            Logger.recordOutput("Drive/skib", "downed speed");
        }).until(getTheRizzFromTheRizzler());
        
    }

    public Command zeroBeerForThePolish() {
        return run(() -> {
            setSigmaSpeed(1, 0);
            setSigmaSpeed(2,0);
            setSigmaSpeed(3, 0);
        }).beforeStarting(() -> {
            Logger.recordOutput("Drive/skib", "zero beero weero speed");
        });

    }

    public Command upTheSpeed() {
        return run(() -> {
            motorOneSpeed.set(motorOneSpeed.get() + 0.005); 
            motorTwoSpeed.set(motorTwoSpeed.get() + 0.005); 
            motorThreeSpeed.set(motorThreeSpeed.get() + 0.005); 
        }).beforeStarting(() -> {
            Logger.recordOutput("Drive/skib", "upped speed");
        });
    }

    public Command downTheSpeed() {
        return run(() -> {
            motorOneSpeed.set(motorOneSpeed.get() - 0.005); 
            motorTwoSpeed.set(motorTwoSpeed.get() - 0.005); 
            motorThreeSpeed.set(motorThreeSpeed.get() - 0.005); 
        }).beforeStarting(() -> {
            Logger.recordOutput("Drive/skib", "downed speed");
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
    Logger.recordOutput("Drive/onGod", motorOneSpeed.get());
   }

    
}

