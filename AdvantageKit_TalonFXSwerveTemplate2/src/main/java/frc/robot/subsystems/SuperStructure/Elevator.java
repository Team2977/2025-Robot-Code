// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
  private final String name;
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert disconected;
  private final Timer stateTimer = new Timer();
  private final PIDController controller;

  private ElevatorState.inputState currentState;
  private ElevatorState.goalState goal = new ElevatorState.goalState(45);

  public Elevator(String name, ElevatorIO io) {
    this.name = name;
    this.io = io;
    this.controller = new PIDController(0, 0, 0);

    disconected = new Alert(name + "Motor Disconected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconected.set(!inputs.leaderConnected);

    if (DriverStation.isEnabled()) {
      currentState = this.io.getState();

      // double effort = this.controller.calculate(currentState.currentHightMeters(),
      // goal.position());
      double effort = RobotContainer.opperator.getRawAxis(1);
      this.io.setState(new ElevatorState.outputState(Optional.of(effort)));
    }
  }

  @AutoLogOutput
  public ElevatorState.inputState getState() {
    return this.currentState;
  }

  public Command setElevatorPose(double position) {
    return new InstantCommand(() -> this.goal = new ElevatorState.goalState(position));
  }

  /*  @AutoLogOutput
  public Command runMotor(double volts) {
    return startEnd(() -> io.runVolts(volts), () -> io.stop());
  }*/
}
