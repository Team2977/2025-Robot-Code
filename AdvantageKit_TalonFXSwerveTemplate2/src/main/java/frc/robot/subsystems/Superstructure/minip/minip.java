// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.minip;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Minip extends SubsystemBase {
  private final String name = "Minip";
  private final MinipIO io;
  private final MinipIOInputsAutoLogged inputs = new MinipIOInputsAutoLogged();
  private final Alert disconected1;
  private final Alert disconected2;
  private final Timer stateTimer = new Timer();

  public Minip(MinipIO io) {
    this.io = io;

    disconected1 = new Alert("ulator Disconected!", Alert.AlertType.kWarning);
    disconected2 = new Alert("ulatorSlave Disconected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconected1.set(!inputs.ulatorConnected);
    disconected2.set(!inputs.ulatorSlaveConnected);
  }

  /*
  @AutoLogOutput
  public Command runUlator(double volts) {
    return startEnd(() -> , null)
  }*/

}
