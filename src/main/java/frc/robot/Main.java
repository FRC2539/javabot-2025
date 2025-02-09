// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.auto.AutoGenerators;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        String generateTrajectories = System.getenv("GENERATE_TRAJECTORIES");
        if ("true".equalsIgnoreCase(generateTrajectories)) {
            RobotBase.startRobot(AutoGenerators.AutoGeneratorRobot::new);
        } else {
            RobotBase.startRobot(Robot::new);
        }
    }
}
