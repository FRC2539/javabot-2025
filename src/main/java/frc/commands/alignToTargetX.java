package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AligningConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class alignToTargetX extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private int targetTag;
    private PIDController xController =
            new PIDController(AligningConstants.Kp, AligningConstants.Ki, AligningConstants.Kd);
    private Vision vision;
    private int cameraID;
    private DoubleSupplier yVelocity = () -> 0;

    // private SwerveRequest.FieldCentricFacingAngle rotateToAngle = new
    // SwerveRequest.FieldCentricFacingAngle();

    public alignToTargetX(
            CommandSwerveDrivetrain drivetrain, Vision vision, int targetTag, int cameraID) {

        this.drivetrain = drivetrain;
        this.targetTag = targetTag;
        this.vision = vision;
        this.cameraID = cameraID;
        addRequirements(drivetrain, vision);
    }

    public alignToTargetX(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            int targetTag,
            int cameraID,
            DoubleSupplier yVelocity) {

        this.drivetrain = drivetrain;
        this.targetTag = targetTag;
        this.vision = vision;
        this.cameraID = cameraID;
        this.yVelocity = yVelocity;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        xController.setTolerance(AligningConstants.aligningDeadband, 0.5);
        xController.setSetpoint(0);
    }

    @Override
    public void execute() {

        double velocity = xController.calculate(vision.getTargetX(cameraID).getRadians());

        drivetrain.setControl(
                drivetrain.driveRobotRelative(
                        new ChassisSpeeds(yVelocity.getAsDouble(), velocity, 0)));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint();
    }

    @Override
    public void end(boolean isInterrupted) {}
}
