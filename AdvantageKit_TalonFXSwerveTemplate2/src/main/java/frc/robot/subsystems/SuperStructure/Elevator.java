// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
  private final String name;
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert disconected;
  private final Timer stateTimer = new Timer();

  public Elevator(String name, ElevatorIO io) {
    this.name = name;
    this.io = io;

    disconected = new Alert(name + "Motor Disconected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconected.set(!inputs.leaderConnected);
  }

  /*  @AutoLogOutput
  public Command runMotor(double volts) {
    return startEnd(() -> io.runVolts(volts), () -> io.stop());
  }*/
}
