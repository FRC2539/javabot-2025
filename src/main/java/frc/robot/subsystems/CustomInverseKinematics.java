package frc.robot.subsystems;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CustomInverseKinematics {
    public class CustomKinematicsSolution {
        public Twist2d estimatedTwist;
    }

  private final SimpleMatrix m_inverseKinematics;
  private final SimpleMatrix[] m_inverseKinematicsMissing;
  private final SimpleMatrix m_forwardKinematics;
  private final SimpleMatrix[] m_forwardKinematicsMissing;

  private final int m_numModules;
  private final Translation2d[] m_modules;
  private Rotation2d[] m_moduleHeadings;
  private Translation2d m_prevCoR = Translation2d.kZero;

  /**
   * Constructs a swerve drive kinematics object. This takes in a variable number of module
   * locations as Translation2d objects. The order in which you pass in the module locations is the
   * same order that you will receive the module states when performing inverse kinematics. It is
   * also expected that you pass in the module states in the same order when calling the forward
   * kinematics methods.
   *
   * @param moduleTranslationsMeters The locations of the modules relative to the physical center of
   *     the robot.
   */
  public CustomInverseKinematics(Translation2d... moduleTranslationsMeters) {
    if (moduleTranslationsMeters.length < 2) {
      throw new IllegalArgumentException("A swerve drive requires at least two modules");
    }
    m_numModules = moduleTranslationsMeters.length;
    m_modules = Arrays.copyOf(moduleTranslationsMeters, m_numModules);
    m_moduleHeadings = new Rotation2d[m_numModules];
    Arrays.fill(m_moduleHeadings, Rotation2d.kZero);
    m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

    for (int i = 0; i < m_numModules; i++) {
      m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
      m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
    }
    
    m_forwardKinematics = m_inverseKinematics.pseudoInverse();

    m_inverseKinematicsMissing = new SimpleMatrix[m_numModules];
    m_forwardKinematicsMissing = new SimpleMatrix[m_numModules];


    for (int i = 0; i < m_numModules; i++) {
      m_inverseKinematicsMissing[i] = m_inverseKinematics.rows(0, i).concatColumns(m_inverseKinematics.rows(i + 1, m_numModules));
      m_forwardKinematicsMissing[i] = m_inverseKinematicsMissing[i].pseudoInverse();
    }

    MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
  }

  public SimpleMatrix toModuleVelocities(SwerveModuleState... moduleStates) {
    var moduleStatesMatrix = new SimpleMatrix(moduleStates.length * 2, 1);

    for (int i = 0; i < m_numModules; i++) {
      var module = moduleStates[i];
      moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
      moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
    }

    return moduleStatesMatrix;
  }

  public SimpleMatrix toModuleVelocities(ChassisSpeeds chassisSpeeds) {
    var chassisSpeedsVector = new SimpleMatrix(3, 1);
    chassisSpeedsVector.set(0, 0, chassisSpeeds.vxMetersPerSecond);
    chassisSpeedsVector.set(1, 0, chassisSpeeds.vyMetersPerSecond);
    chassisSpeedsVector.set(2, 0, chassisSpeeds.omegaRadiansPerSecond);

    return m_inverseKinematics.mult(chassisSpeedsVector);
  }

  public ChassisSpeeds toChassisSpeeds(int missingModule, SwerveModuleState... moduleStates) {
    if (moduleStates.length != m_numModules - 1) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of module locations provided in "
              + "constructor");
    }
    var moduleStatesMatrix = new SimpleMatrix((m_numModules - 1) * 2, 1);

    for (int i = 0; i < m_numModules - 1; i++) {
      var module = moduleStates[i];
      moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
      moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
    }

    var chassisSpeedsVector = m_forwardKinematicsMissing[missingModule].mult(moduleStatesMatrix);
    return new ChassisSpeeds(
        chassisSpeedsVector.get(0, 0),
        chassisSpeedsVector.get(1, 0),
        chassisSpeedsVector.get(2, 0));
  }

  public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
    if (moduleStates.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of module locations provided in "
              + "constructor");
    }
    var moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

    for (int i = 0; i < m_numModules; i++) {
      var module = moduleStates[i];
      moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
      moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
    }

    var chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
    return new ChassisSpeeds(
        chassisSpeedsVector.get(0, 0),
        chassisSpeedsVector.get(1, 0),
        chassisSpeedsVector.get(2, 0));
  }

  /**
   * Performs forward kinematics to return the resulting chassis state from the given module states.
   * This method is often used for odometry -- determining the robot's position on the field using
   * data from the real-world speed and angle of each module on the robot.
   *
   * @param moduleDeltas The latest change in position of the modules (as a SwerveModulePosition
   *     type) as measured from respective encoders and gyros. The order of the swerve module states
   *     should be same as passed into the constructor of this class.
   * @return The resulting Twist2d.
   */
  public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {
    if (moduleDeltas.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of module locations provided in "
              + "constructor");
    }
    var moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);

    for (int i = 0; i < m_numModules; i++) {
      var module = moduleDeltas[i];
      moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
      moduleDeltaMatrix.set(i * 2 + 1, module.distanceMeters * module.angle.getSin());
    }

    var chassisDeltaVector = m_forwardKinematics.mult(moduleDeltaMatrix);
    return new Twist2d(
        chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
  }

  public Twist2d toTwist2d(int missingModule, SwerveModulePosition... moduleDeltas) {
    if (moduleDeltas.length != m_numModules - 1) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of module locations provided in "
              + "constructor");
    }
    var moduleDeltaMatrix = new SimpleMatrix((m_numModules - 1) * 2, 1);

    for (int i = 0; i < (m_numModules - 1); i++) {
      var module = moduleDeltas[i];
      moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
      moduleDeltaMatrix.set(i * 2 + 1, module.distanceMeters * module.angle.getSin());
    }

    var chassisDeltaVector = m_forwardKinematicsMissing[missingModule].mult(moduleDeltaMatrix);
    return new Twist2d(
        chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
  }

  public Twist2d toTwist2d(SwerveModulePosition[] start, SwerveModulePosition[] end) {
    if (start.length != end.length) {
      throw new IllegalArgumentException("Inconsistent number of modules!");
    }
    var newPositions = new SwerveModulePosition[start.length];
    for (int i = 0; i < start.length; i++) {
      newPositions[i] =
          new SwerveModulePosition(end[i].distanceMeters - start[i].distanceMeters, end[i].angle);
    }
    return toTwist2d(newPositions);
  }

  public Twist2d toTwist2d(int missingModule, SwerveModulePosition[] start, SwerveModulePosition[] end) {
    if (start.length != end.length) {
      throw new IllegalArgumentException("Inconsistent number of modules!");
    }
    var newPositions = new SwerveModulePosition[start.length];
    for (int i = 0; i < start.length; i++) {
      newPositions[i] =
          new SwerveModulePosition(end[i].distanceMeters - start[i].distanceMeters, end[i].angle);
    }
    return toTwist2d(missingModule, newPositions);
  }
}
