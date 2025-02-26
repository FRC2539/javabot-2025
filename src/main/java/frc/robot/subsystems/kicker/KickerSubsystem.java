// package frc.robot.subsystems.kicker;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// public class KickerSubsystem extends SubsystemBase {

//     private KickerIO piviotIO;

//     private KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

//     // NetworkTableInstance nInstance = NetworkTableInstance.getDefault();
//     // NetworkTable table = nInstance.getTable("SmartDashboard");
//     // NetworkTableValue grippervoltage = table.getValue("grippervoltage");

//     LoggedNetworkNumber kickervoltage = new LoggedNetworkNumber("Kicker Voltage", 0);

//     public KickerSubsystem(KickerIO armrollerIO) {
//         this.piviotIO = armrollerIO;
//     }

//     public void periodic() {
//         piviotIO.updateInputs(kickerInputs);
//         Logger.processInputs("RealOutputs/Kicker", kickerInputs);
//     }

//     public Command setPosition(double position) {

//         while (kickerInputs.position != position) {
//             return run(() -> piviotIO.setVoltage(12));
//         }

//         return runOnce(() -> piviotIO.setVoltage(0));
//     }
// }
