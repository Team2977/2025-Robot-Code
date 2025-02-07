// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import java.util.Optional;

/** Add your docs here. */
public class ElevatorState {
  public record inputState(double currentHightMeters, double currentVelocityMetersPerSec) {}

  public record outputState(Optional<Double> voltage) {}

  public record goalState(double position) {}
}
