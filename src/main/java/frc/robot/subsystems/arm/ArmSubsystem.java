package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    private ArmPivotIO armPivotIO;
    public ArmPivotIOInputsAutoLogged armPivotInputs = new ArmPivotIOInputsAutoLogged();

    private PIDController controller = new PIDController(0.1, 0, 0);
    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));

    private double reference = 0;
    private TrapezoidProfile.State state = new TrapezoidProfile.State(0,0);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);

    public ArmSubsystem(ArmPivotIO armPivotIO) {
        this.armPivotIO = armPivotIO;
        setDefaultCommand(setVoltage(0));
    }

    public void periodic() {
        armPivotIO.updateInputs(armPivotInputs);
        Logger.processInputs("RealOutputs/Arm", armPivotInputs);
    }

    public Command setVoltage(double voltage) {
        return run(() -> armPivotIO.setVoltage(voltage));
    }

    public Command armPivotUp() {
        return setVoltage(12);
    }

    public Command armpivotDown() {
        return setVoltage(-12);
    }

    public Command setPosition(double arm) {
        return startRun(
            () -> {
                reference = arm;
            },
            () -> {
                double voltage = controller.calculate(armPivotInputs.throughboreEncoderPosition, reference);
                voltage = Math.min(12.0, Math.max(-12.0, voltage)); // Clamp voltage
                armPivotIO.setVoltage(voltage);
            }
        );
    }


    private Command profileToReference() {
        return startRun(
            () -> {
                goal = new TrapezoidProfile.State(reference, 0);
                state = new TrapezoidProfile.State(armPivotInputs.position, armPivotInputs.velocity);
            }, () -> {
                double voltage = controller.calculate(armPivotInputs.position, state.position);
                voltage = Math.min(12.0, Math.max(-12.0, voltage)); // Clamp voltage
                armPivotIO.setVoltage(voltage);
                if (profile.timeLeftUntil(goal.position) <= 0.25) {

                };
                state = profile.calculate(0.02, state, goal);
            });
    }

    private Command pidLoopAtReference() {

    }

    public double getArmPosition() {
        return armPivotInputs.position;
    }

    public Command holdArmPosition() {
        return startRun(
            () -> {
                reference = armPivotInputs.position;
            }, 
            () -> {

            }
        );
    }
}
