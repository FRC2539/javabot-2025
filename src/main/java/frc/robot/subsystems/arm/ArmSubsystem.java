package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ArmConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmSubsystem extends SubsystemBase {
    private ArmPivotIO armPivotIO;
    public ArmPivotIOInputsAutoLogged armPivotInputs = new ArmPivotIOInputsAutoLogged();
    public LoggedNetworkNumber armTuneables = new LoggedNetworkNumber("arm tuneable", 9);

    private PIDController controller =
            new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);

    private double reference = 0;

    private SysIdRoutine armSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            state ->
                                    Logger.recordOutput(
                                            "Elevator/SysIdArm_State", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (voltage) -> armPivotIO.setVoltage(voltage.in(Volts)), null, this));

    public ArmSubsystem(ArmPivotIO armPivotIO) {
        this.armPivotIO = armPivotIO;
        controller.setTolerance(ArmConstants.ARM_TOLERANCE);
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {
        armPivotIO.updateInputs(armPivotInputs);
        Logger.processInputs("RealOutputs/Arm", armPivotInputs);
    }

    public Command runQStaticArmSysId(SysIdRoutine.Direction direction) {
        return armSysIdRoutine.quasistatic(direction);
    }

    public Command runDynamicArmSysId(SysIdRoutine.Direction direction) {
        return armSysIdRoutine.dynamic(direction);
    }

    public Command setVoltage(double voltage) {
        return run(() -> armPivotIO.setVoltage(voltage));
    }

    public Command armpivotUp() {
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
        if (position > ArmConstants.upperLimit) {
            position = ArmConstants.upperLimit;
        }
        if (position < ArmConstants.lowerLimit) {
            position = ArmConstants.lowerLimit;
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
