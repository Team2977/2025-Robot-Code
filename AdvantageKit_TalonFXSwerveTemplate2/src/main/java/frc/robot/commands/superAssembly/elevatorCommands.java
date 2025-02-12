// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superAssembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure.Elevator;

/** Add your docs here. */
public class elevatorCommands {

  private elevatorCommands() {}

  public static Command moveToPositionSIM(Elevator elevator, double goal) {

    return Commands.run(
            () -> {
              elevator.runToPosition(goal);
              SmartDashboard.putBoolean("working????", true);
            },
            elevator)
        // .finallyDo(() -> elevator.runToPosition(0))
        .finallyDo(() -> SmartDashboard.putBoolean("working????", false));
  }
}
