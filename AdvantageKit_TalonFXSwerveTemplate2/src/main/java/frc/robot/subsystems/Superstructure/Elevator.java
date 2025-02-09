// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
  private final String name;
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert disconected;
  private final Timer stateTimer = new Timer();
  private final ProfiledPIDController controller;

  public Elevator(String name, ElevatorIO io) {
    this.name = name;
    this.io = io;
    this.controller =
        new ProfiledPIDController(
            0, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // TODO set diffrent numbers here

    disconected = new Alert(name + "Motor Disconected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }
}
