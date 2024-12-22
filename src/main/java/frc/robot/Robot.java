// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        Notifier m_simNotifier;

        m_robotContainer = new RobotContainer();

        Logger.recordMetadata("ProjectName", "JavaBot-2025"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            setUseTiming(true);
            PowerDistribution distribution =
                    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(true); // Run at a normal speed
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

            SimulatedArena.overrideSimulationTimings(Seconds.of(0.02 / 5), 1);

            /* Run simulation at a faster rate so PID gains behave more reasonably */
            m_simNotifier =
                    new Notifier(
                            () -> {
                                SimulatedArena.getInstance().simulationPeriodic();
                            });
            m_simNotifier.startPeriodic(0.02 / 5);

            m_robotContainer.drivetrain.resetPose(new Pose2d(1, 2, new Rotation2d()));
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata
        // values may
        // be added.
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.auto.logAutoInformation();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        // RoboRioSim.setVInVoltage(12.0);
        // RobotController.getBatteryVoltage();
        // RobotController.getInputVoltage();
        // driveSim.setSimulationWorldPose(m_robotContainer.drivetrain.getState().Pose);

        // m_robotContainer.drivetrain.resetPose(driveSim.getActualPoseInSimulationWorld());

        List<Pose3d> notesPoses = SimulatedArena.getInstance().getGamePiecesByType("Note");

        // Publish to telemetry using AdvantageKit
        Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses.toArray(new Pose3d[0]));
    }
}
