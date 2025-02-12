package frc.robot.subsystems.Superstructure;

import java.util.Optional;

public class ElevatorState {

  public record inputState(double currentPosition, double currentVel) {}

  public record outputState(Optional<Double> position) {}

  public record goalState(double position) {}
}
