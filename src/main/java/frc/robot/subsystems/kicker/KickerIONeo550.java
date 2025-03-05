// package frc.robot.subsystems.kicker;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import frc.robot.constants.ChuteConstants;

// public class KickerIONeo550 implements KickerIO {
//     private SparkMax kickerMotor = new SparkMax(900, SparkLowLevel.MotorType.kBrushless);

//     public KickerIONeo550() {
//         kickerMotor.getEncoder().setPosition(0);

//         SparkBaseConfig config =
//                 new SparkMaxConfig()
//                         .smartCurrentLimit((int) ChuteConstants.ChuteCurrent)
//                         .secondaryCurrentLimit(ChuteConstants.ChuteCurrent)
//                         .idleMode(IdleMode.kBrake);
//         kickerMotor.configure(
//                 config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     public void updateInputs(KickerIOInputs inputs) {
//         inputs.position = kickerMotor.getEncoder().getPosition();

//         inputs.voltage = kickerMotor.getBusVoltage();
//     }

//     public void setVoltage(double voltage) {
//         kickerMotor.setVoltage(voltage);
//     }
// }
