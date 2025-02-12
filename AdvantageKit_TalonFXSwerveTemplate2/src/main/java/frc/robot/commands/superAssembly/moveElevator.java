// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superAssembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure.Elevator;
import frc.robot.subsystems.Superstructure.constantsE;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveElevator extends Command {
  /** Creates a new moveElevator. */
  public Elevator elevator;

  public double goal;

  public moveElevator(Elevator elevator, double goal) {
    this.elevator = elevator;
    this.goal = goal;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("COMMAND WORKING", true);
  }

  @Override
  public void execute() {

    constantsE.GoalElevator = goal;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("COMMAND WORKING", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
