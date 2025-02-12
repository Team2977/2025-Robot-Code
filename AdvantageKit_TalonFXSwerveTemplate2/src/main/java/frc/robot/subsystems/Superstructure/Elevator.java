// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert disconected;
  private final Timer stateTimer = new Timer();
  private final ProfiledPIDController controller;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.controller =
        new ProfiledPIDController(
            0, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // TODO set diffrent numbers here

    disconected = new Alert("elevator" + "Motor Disconected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.updateTelemetry();
    if (Constants.currentMode == Mode.SIM) {
      io.simulationPeriodic();
    }
  }
  // TODO add runToPosition here. look into what is required to make it work. do I need
  // InstantCommand.run() ?

  @AutoLogOutput
  public Command runToPosition(double goal) {
    // return runEnd(() -> io.runToPosition(goal), () -> io.stop());

    return runEnd(
        () -> SmartDashboard.putBoolean("got here", true),
        () -> SmartDashboard.putBoolean("got here", false));
  }
}
