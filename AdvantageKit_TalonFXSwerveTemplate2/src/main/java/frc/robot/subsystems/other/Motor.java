// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.other;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Motor extends SubsystemBase {
  private final String name;
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private final Alert disconected;
  private final Timer stateTimer = new Timer();

  public Motor(String name, MotorIO io) {
    this.name = name;
    this.io = io;

    disconected = new Alert(name + "Motor Disconected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconected.set(!inputs.connected);
  }

  @AutoLogOutput
  public Command runMotor(double volts) {
    return startEnd(() -> io.runVolts(volts), () -> io.stop());
  }
}
