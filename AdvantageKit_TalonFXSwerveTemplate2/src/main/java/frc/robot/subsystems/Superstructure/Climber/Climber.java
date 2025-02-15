// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.Climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Climber extends SubsystemBase {
  private final String name = "Climber";
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert disconected;
  private final Timer stateTimer = new Timer();

  public Climber(ClimberIO io) {

    this.io = io;

    disconected = new Alert(name + "Motor Disconected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconected.set(!inputs.connected);
  }
}
