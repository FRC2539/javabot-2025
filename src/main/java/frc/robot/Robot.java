// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();

        Logger.recordMetadata("ProjectName", "JavaBot-2025"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            PowerDistribution distribution =
                    new PowerDistribution(
                            33, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(true); // Run as fast as possible
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

            try {
                Path path =
                        Paths.get(Filesystem.getDeployDirectory().getPath(), "elastic-layout.json");
                Charset charset = StandardCharsets.UTF_8;

                String content = new String(Files.readAllBytes(path), charset);

                content =
                        content.replaceAll(
                                "\"/CameraPublisher/([^/\"]+)", "\"/CameraPublisher/$1-processed");
                Path outPath =
                        Paths.get(
                                Filesystem.getDeployDirectory().getPath(),
                                "elastic-layout-sim.json");
                Files.write(outPath, content.getBytes(charset));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata
        // values may be added.

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
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

        Elastic.selectTab("Teleoperated");

        Elastic.sendNotification(
                new Notification(
                        Notification.NotificationLevel.INFO,
                        "You're enabled!",
                        "You can drive around now!"));
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
    public void simulationPeriodic() {}
}
