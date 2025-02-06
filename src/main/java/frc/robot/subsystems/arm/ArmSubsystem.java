package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

public class ArmSubsystem extends SubsystemBase {
    private ArmPivotIO armPivotIO;
    public ArmPivotIOInputsAutoLogged armPivotInputs = new ArmPivotIOInputsAutoLogged();

    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    
    private SysIdRoutine armSysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),  
            new SysIdRoutine.Mechanism((voltage) -> armPivotIO.setVoltage(voltage.in(Volts)), null, this));

     private SysIdRoutine wristSysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),  
            new SysIdRoutine.Mechanism((voltage) -> wristIO.setVoltage(voltage.in(Volts)), null, this));

    public ArmSubsystem(ArmPivotIO armPivotIO, WristIO wristIO) {
        this.armPivotIO = armPivotIO;
        this.wristIO = wristIO;

        setDefaultCommand(
                run(
                        () -> {
                            armPivotIO.setPosition(0);
                            wristIO.setPosition(0);
                        }));
    }

    public void periodic() {
        armPivotIO.updateInputs(armPivotInputs);
        wristIO.updateInputs(wristInputs);

        wristIO.encoderUpdate();
        Logger.processInputs("RealOutputs/Arm", armPivotInputs);
        Logger.processInputs("RealOutputs/Wrist", wristInputs);
    }

    public Command runQStaticArmSysId(SysIdRoutine.Direction direction) {
        return armSysIdRoutine.quasistatic(direction);
    }

    public Command runDynamicArmSysId(SysIdRoutine.Direction direction) {
        return armSysIdRoutine.dynamic(direction);
    }

    public Command runQStaticWristSysId(SysIdRoutine.Direction direction) {
        return wristSysIdRoutine.quasistatic(direction);
    }

    public Command runDynamicWristSysId(SysIdRoutine.Direction direction) {
        return wristSysIdRoutine.dynamic(direction);
    }
    
    public Command turnWristRight() {
        return setVoltageWrist(12);
    }

    public Command turnWristLeft() {
        return setVoltageWrist(-12);
    }

    public Command armPivotUp() {
        return setVoltageArm(12);
    }

    public Command armpivotDown() {
        return setVoltageArm(-12);
    }

    public Command setVoltageArm(double voltage) {
        return run(
                () -> {
                    armPivotIO.setVoltage(voltage);
                });
    }

    public Command setVoltageWrist(double voltage) {
        return run(
                () -> {
                    wristIO.setVoltage(voltage);
                });
    }

    public Command setPosition(double wrist, double arm) {
        return run(
                () -> {
                    armPivotIO.setPosition(arm);
                    wristIO.setPosition(wrist);
                });
    }

    public double getArmPosition() {
        return armPivotIO.getPosition();
    }

    public double getWristPosition() {
        return wristIO.getPosition();
    }
}
