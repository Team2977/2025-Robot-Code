// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.minip;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface MinipIO {

  @AutoLog
  class MinipIOInputs {
    public boolean ulatorConnected = false;
    public double ulatorPositionRota = 0.0;
    public double ulatorVelocityRPM = 0.0;
    public double ulatorAppliedVoltage = 0.0;
    public double ulatorOutputAmps = 0.0;
    public double ulatorTempCelsius = 0.0;

    public boolean ulatorSlaveConnected = false;
    public double ulatorSlavePositionRota = 0.0;
    public double ulatorSlaveVelocityRPM = 0.0;
    public double ulatorSlaveAppliedVoltage = 0.0;
    public double ulatorSlaveOutputApms = 0.0;
    public double ulatorSlaveTempCelsius = 0.0;
  }

  default void updateInputs(MinipIOInputs inputs) {}

  default void minipIntake() {}

  default void minipSpit() {}

  default void stopMotors() {}
}
