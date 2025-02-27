// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.other;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  @AutoLog
  public static class MotorIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(MotorIOInputs inputs) {}

  // Set run volts
  default void runVolts(double volts) {}

  // Stop motor
  default void stop() {}
}
