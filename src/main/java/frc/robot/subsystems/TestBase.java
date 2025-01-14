package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TestBase extends SubsystemBase {
    public LoggedNetworkNumber motor1SpeedNetworkNumber =
            new LoggedNetworkNumber("motorOneSpeed", 0.1);
    public LoggedNetworkNumber motor2SpeedNetworkNumber =
            new LoggedNetworkNumber("motorTwoSpeed", 0.1);

    public SparkMax motor1 = new SparkMax(2, MotorType.kBrushless);
    public SparkMax motor2 = new SparkMax(3, MotorType.kBrushless);
    // run motors 1/2 forward and backwards at a certain speed.
    private final Timer startTimer = new Timer();
    private boolean stop1, stop2, stop3;
    LoggedNetworkBoolean simulation = new LoggedNetworkBoolean("isSimulation", false);

    boolean trueOrFalse;
    public final Trigger trigger2 = new Trigger(() -> simulation.get());
    // change cansparkmax motors

    Trigger trigger;

    public TestBase() {
        // boolean stop1, stop2, stop3 = false;
        // startTimer.reset();\
        SparkBaseConfig test = new SparkMaxConfig().smartCurrentLimit(20).secondaryCurrentLimit(20).idleMode(IdleMode.kBrake);
        motor1.configure(test, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(test, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotor1Speed(double speed) {
        motor1.set(speed);
    }

    public void setMotor2Speed(double speed) {
        motor2.set(speed);
    }

    public Command move() {
        return run(
                () -> {
                    setSpeed();
                });
    }

    public void setSpeed() {
        motor1.set(motor1SpeedNetworkNumber.get());
        motor2.set(motor2SpeedNetworkNumber.get());
    }

    public Command moveForwardMotor1() {
        return run(
                () -> {
                    motor1.set(1);
                });
    }

    public Command moveForwardMotor2() {
        return run(
                () -> {
                    motor2.set(1);
                });
    }

    public Command moveBackwardMotor1() {

        return run(
                () -> {
                    motor1.set(-1);
                });
    }

    public Command moveBackwardMotor2() {

        return run(
                () -> {
                    motor2.set(-1);
                });
    }

    public Command stop() {
        return run(
                () -> {
                    motor1.set(0);
                    motor2.set(0);
                });
    }

    public boolean simulationGetBoolean() {
        return simulation.get();
    }

    //     return run(() ->{

    //         while(!startTimer.hasElapsed(1) && (!stop1)){
    //         motor1.set(1);
    //         motor2.set(1);
    //         }

    //         startTimer.restart();
    //         stop1 = true;

    //         while(!startTimer.hasElapsed(1) && (!stop2)){
    //         stop();
    //         }

    //         startTimer.reset();
    //         stop2 = true;

    //         while(!startTimer.hasElapsed(1) && (!stop3)){
    //         setSpeed();
    //         move();
    //         }
    //         stop3 = true;
    //         stop();

    //     });

    public Command runSpeedsV2() {
        return move().withTimeout(1)
                .andThen(stop().withTimeout(1).andThen(move().withTimeout(1)))
                .repeatedly();
    }
}
