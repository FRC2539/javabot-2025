package frc.robot.subsystems.TestBase;

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
import frc.robot.subsystems.TestBase.MotorIO.MotorIOInputs;

public class TestBaseSubsystem extends SubsystemBase{
    LoggedNetworkNumber motorOneSpeed = new LoggedNetworkNumber("motorOne", 1);
    LoggedNetworkNumber motorTwoSpeed = new LoggedNetworkNumber("motorTwo", 1);
    LoggedNetworkNumber motorThreeSpeed = new LoggedNetworkNumber("motorThree", 1);
    
    
    private SensorIO sensor;
  
    
    
    
    MotorIOInputs motorOneInputs = new MotorIOInputs();
    private final MotorIO motorOneIO;

    MotorIOInputs motorTwoInputs = new MotorIOInputs();
    private final MotorIO motorTwoIO;

    MotorIOInputs motorThreeInputs = new MotorIOInputs();
    private final MotorIO motorThreeIO;

    public TestBaseSubsystem(MotorIO motorOne, MotorIO motorTwo, MotorIO motorThree, SensorIO sensor) {
        this.motorOneIO = motorOne;
        this.motorTwoIO = motorTwo;
        this.motorThreeIO = motorThree;
        this.sensor = sensor;

    }

 

    
    private void setSpeedAmount(MotorIO motor, double speed){
        motor.setMotorSpeed(speed);
    }

    

    public Command run() {
        return run(() -> {
            setSpeedAmount(motorOneIO, motorOneSpeed.get());
            setSpeedAmount(motorTwoIO,motorTwoSpeed.get());
            setSpeedAmount(motorThreeIO, motorThreeSpeed.get());
        
        })
        .beforeStarting(() -> {
            Logger.recordOutput("Drive/driving", "running");
        }).until(sensor.getValue());
        
    }

    public BooleanSupplier getGet(){
        
        return sensor.getValue();
    }
    //Mostly works, however when button is released it instead disables the main motors instead of the followers. Basically, it should disable after the left dpad is dropped and x is held, but it actually does the opposite.


    public Command noSpeed() {
        return run(() -> {
            setSpeedAmount(motorOneIO, 0);
            setSpeedAmount(motorTwoIO,0);
            setSpeedAmount(motorThreeIO, 0);
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
    motorOneIO.updateInputs(motorOneInputs);
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
    Logger.recordOutput("Drive/motor found speed", motorOneIO.getMotorSpeed());
   }

    
}

