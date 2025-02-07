// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superAssembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperStructure.Elevator;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class elevatorCommands {

  private elevatorCommands() {}

  public static Command elevatorUp(Elevator elevator, DoubleSupplier pose) {
    return Commands.run(
            () -> {
              elevator.setElevatorPose(10);
              SmartDashboard.putBoolean("riningh", true);
            },
            elevator)
        .finallyDo(() -> SmartDashboard.putBoolean("riningh", false));
  }
}
