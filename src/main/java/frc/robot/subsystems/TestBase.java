package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.Scanner; 



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TestBase extends SubsystemBase {
    public LoggedNetworkNumber motor1SpeedNetworkNumber =
            new LoggedNetworkNumber("wristSpeed", 0.1);
    public LoggedNetworkNumber motor2SpeedNetworkNumber =
            new LoggedNetworkNumber("gripperSpeed", 0.7);
    public LoggedNetworkNumber motor3SpeedNetworkNumber =
            new LoggedNetworkNumber("motorThreeSpeed", 0.1);

    public TalonFX motor3 = new TalonFX(9);
    // run motors 1/2 forward and backwards at a certain speed.
    //private final Timer startTimer = new Timer();
    //private boolean stop1, stop2, stop3;
    LoggedNetworkBoolean simulation = new LoggedNetworkBoolean("isSimulation", false);
    
    boolean trueOrFalse;
    public final Trigger trigger2 = new Trigger(() -> simulation.get());
    // change cansparkmax motors

    Trigger trigger;

    public TestBase() {
        // boolean stop1, stop2, stop3 = false; 
        // startTimer.reset();\
        
        motor3.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(120));
        motor3.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        //motor2.configure(test, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotor3Speed(double speed) {
        motor3.set(speed);
    }

    

    public boolean simulationGetBoolean() {
        return simulation.get();
    }
}
