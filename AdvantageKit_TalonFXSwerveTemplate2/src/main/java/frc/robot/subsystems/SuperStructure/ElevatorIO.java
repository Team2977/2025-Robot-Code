// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public double leaderPositionRads = 0.0;
    public double leaderVelocityRadsPerSec = 0.0;
    public double leaderAppliedVoltage = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public boolean followerConnected = false;
    public double followerPositionRads = 0.0;
    public double followerVelocityRadsPerSec = 0.0;
    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  // Set run volts
  default void runVoltsElevator(double volts) {}

  // Stop motor
  default void stopElevator() {}
}
