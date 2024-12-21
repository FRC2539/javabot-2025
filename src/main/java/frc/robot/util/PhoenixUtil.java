// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

public final class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        private static int instances = 0;
        public final int id;

        private final TalonFXSimState talonFXSimState;

        public TalonFXMotorControllerSim(TalonFX talonFX, boolean motorInverted) {
            this.id = instances++;

            this.talonFXSimState = talonFX.getSimState();
            talonFXSimState.Orientation =
                    motorInverted
                            ? ChassisReference.Clockwise_Positive
                            : ChassisReference.CounterClockwise_Positive;
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(12.0);
            Logger.recordOutput(
                    "CTREMotor/" + id + "/mechanismAngleRad", mechanismAngle.in(Radians));
            Logger.recordOutput(
                    "CTREMotor/" + id + "/mechanismVelRadPerSec",
                    mechanismVelocity.in(RadiansPerSecond));
            Logger.recordOutput("CTREMotor/" + id + "/encoderAngleRad", encoderAngle.in(Radians));
            Logger.recordOutput(
                    "CTREMotor/" + id + "/encoderVelRadPerSec",
                    encoderVelocity.in(RadiansPerSecond));
            Voltage output = talonFXSimState.getMotorVoltageMeasure();
            Logger.recordOutput("CTREMotor/" + id + "/outputVoltage", output.in(Volts));
            return output;
        }
    }

    public static class TalonFXMotorControllerWithRemoteCancoderSim
            implements SimulatedMotorController {
        private final CANcoderSimState remoteCancoderSimState;
        private final Angle encoderOffset;

        public final int id;

        private final TalonFXSimState talonFXSimState;

        public TalonFXMotorControllerWithRemoteCancoderSim(
                TalonFX talonFX,
                boolean motorInverted,
                CANcoder cancoder,
                boolean encoderInverted,
                Angle encoderOffset) {
            this.id = TalonFXMotorControllerSim.instances++;

            this.talonFXSimState = talonFX.getSimState();
            talonFXSimState.Orientation =
                    motorInverted
                            ? ChassisReference.Clockwise_Positive
                            : ChassisReference.CounterClockwise_Positive;

            this.remoteCancoderSimState = cancoder.getSimState();
            this.remoteCancoderSimState.Orientation =
                    encoderInverted
                            ? ChassisReference.Clockwise_Positive
                            : ChassisReference.CounterClockwise_Positive;
            this.encoderOffset = encoderOffset;
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(mechanismAngle.minus(encoderOffset));
            talonFXSimState.setRotorVelocity(mechanismVelocity);
            talonFXSimState.setSupplyVoltage(12.0);
            Logger.recordOutput(
                    "CTREMotor/" + id + "/mechanismAngleRad", mechanismAngle.in(Radians));
            Logger.recordOutput(
                    "CTREMotor/" + id + "/mechanismVelRadPerSec",
                    mechanismVelocity.in(RadiansPerSecond));
            Logger.recordOutput("CTREMotor/" + id + "/encoderAngleRad", encoderAngle.in(Radians));
            Logger.recordOutput(
                    "CTREMotor/" + id + "/encoderVelRadPerSec",
                    encoderVelocity.in(RadiansPerSecond));
            remoteCancoderSimState.setRawPosition(mechanismAngle.minus(encoderOffset));
            remoteCancoderSimState.setVelocity(mechanismVelocity);
            Voltage output = talonFXSimState.getMotorVoltageMeasure();
            Logger.recordOutput("CTREMotor/" + id + "/outputVoltage", output.in(Volts));

            return output;
        }
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps =
                new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] =
                    Timer.getFPGATimestamp()
                            - 0.02
                            + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
