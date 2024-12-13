package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveIO {
    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request);

    /**
     * Resets the pose of the robot. The pose should be from the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     *
     * @param pose Pose to make the current pose
     */
    public void resetPose(Pose2d pose);

    /**
     * Resets the rotation of the robot pose to 0 from the
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}
     * perspective. This makes the current orientation of the robot X
     * forward for field-centric maneuvers.
     * <p>
     * This is equivalent to calling {@link #resetRotation} with the operator
     * perspective rotation.
     */
    public void seedFieldCentric();

    /**
     * Resets the rotation of the robot pose without affecting translation.
     * The rotation should be from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance}
     * perspective.
     *
     * @param rotation Rotation to make the current rotation
     */
    public void resetRotation(Rotation2d rotation);

    /**
     * Takes the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective direction
     * and treats it as the forward direction for
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
     * <p>
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     * <p>
     * This does not change the robot pose, which is in the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     * As a result, the robot pose may need to be reset using {@link #resetPose}.
     *
     * @param fieldDirection Heading indicating which direction is forward from
     *                       the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective
     */
    public void setOperatorPerspectiveForward(Rotation2d fieldDirection);

    /**
     * Updates all the simulation state variables for this
     * drivetrain class. User provides the update variables for the simulation.
     *
     * @param dtSeconds time since last update call
     * @param supplyVoltage voltage as seen at the motor controllers
     */
    public void updateSimState(double dtSeconds, double supplyVoltage);

    /**
     * Gets the current state of the swerve drivetrain.
     * This includes information such as the pose estimate,
     * module states, and chassis speeds.
     *
     * @return Current state of the drivetrain
     */
    public SwerveDriveState getState();

    /**
     * Gets this drivetrain's Pigeon 2 reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This drivetrain's Pigeon 2 reference
     */
    public Pigeon2 getPigeon2();
}
