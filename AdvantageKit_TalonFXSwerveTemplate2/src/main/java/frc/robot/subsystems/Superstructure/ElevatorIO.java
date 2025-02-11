// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public double leaderPositionRota = 0.0;
    public double leaderVelocityRotaPerSec = 0.0;
    public double leaderAppliedVoltage = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public boolean followerConnected = false;
    public double followerPositionRota = 0.0;
    public double followerVelocityRotaPerSec = 0.0;
    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;

    public double goal = 0.0;
    public double PIDControllerOutput = 0.0;
    public double feedforwardOutput = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void updateTelemetry() {}

  default void runToPosition(double goal) {}
  // for REAL elevator. the sim elevator uses meters as the goal units. for the REAL one it must be
  // in motor rotations
  default void REALrunToPosition(double goal) {}
}
