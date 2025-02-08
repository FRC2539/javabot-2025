package frc.robot.subsystems.arm;

<<<<<<< HEAD
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
=======
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;
>>>>>>> main
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmSubsystem extends SubsystemBase {
    private ArmPivotIO armPivotIO;
    public ArmPivotIOInputsAutoLogged armPivotInputs = new ArmPivotIOInputsAutoLogged();
    public LoggedNetworkNumber armTuneables = new LoggedNetworkNumber("arm tuneable", 9);

    private PIDController controller =
            new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);

<<<<<<< HEAD
    private SysIdRoutine armSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            state -> Logger.recordOutput("SysIdElevator_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (voltage) -> armPivotIO.setVoltage(voltage.in(Volts)), null, this));

    private SysIdRoutine wristSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            state ->
                                    SignalLogger.writeString(
                                            "SysIdElevator_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (voltage) -> wristIO.setVoltage(voltage.in(Volts)), null, this));

    public ArmSubsystem(ArmPivotIO armPivotIO, WristIO wristIO) {
=======
    private double reference = 0;

    public ArmSubsystem(ArmPivotIO armPivotIO) {
>>>>>>> main
        this.armPivotIO = armPivotIO;
        controller.setTolerance(ArmConstants.ARM_TOLERANCE);
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {
        armPivotIO.updateInputs(armPivotInputs);
        Logger.processInputs("RealOutputs/Arm", armPivotInputs);
    }

<<<<<<< HEAD
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
=======
    public Command setVoltage(double voltage) {
        return run(() -> armPivotIO.setVoltage(voltage));
>>>>>>> main
    }

    public Command armPivotUp() {
        return setVoltage(12);
    }

    public Command armpivotDown() {
        return setVoltage(-12);
    }

    public Command tuneableVoltage() {
        return run(() -> setVoltage(armTuneables.get()));
    }

    public Command tunablePose() {
        return runOnce(() -> reference = armTuneables.get()).andThen(followReference());
    }

    public Command setPosition(double position) {
        if (position > WristConstants.upperLimit) {
            position = WristConstants.upperLimit;
        }
        if (position < WristConstants.lowerLimit) {
            position = WristConstants.lowerLimit;
        }
        double nextPosition = position;

        return runOnce(
                        () -> {
                            reference = nextPosition;
                        })
                .andThen(followReference());
    }

    public double getPosition() {
        return armPivotInputs.throughboreEncoderPosition;
    }

    private Command followReference() {
        return run(
                () -> {
                    double voltage =
                            controller.calculate(
                                    armPivotInputs.throughboreEncoderPosition, reference);
                    if (controller.atSetpoint()) {
                        voltage = 0;
                    } else {
                        voltage = Math.min(12.0, Math.max(-12.0, voltage)); // Clamp voltage
                    }
                    armPivotIO.setVoltage(voltage);
                });
    }

    public double getInternalEncoderPosition() {
        return armPivotInputs.position;
    }

    public boolean isEncoderConnected() {
        return armPivotInputs.throughboreConnected;
    }
}
