// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperStructure.elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveElevator extends Command {
  private final ProfiledPIDController controller =
      new ProfiledPIDController(0.25, 0.05, 0, new TrapezoidProfile.Constraints(100, 200));
  private elevator ELEVATOR;
  private double Goal;
  private boolean finishCommand;

  public moveElevator(elevator ELEVATOR, double Goal) {
    this.ELEVATOR = ELEVATOR;
    this.Goal = Goal;
    addRequirements(ELEVATOR);
  }

  @Override
  public void initialize() {
    controller.reset(ELEVATOR.leader.getPosition().getValueAsDouble());
    finishCommand = false;
  }

  @Override
  public void execute() {

    Constants.elevatorGoal =
        controller.calculate(ELEVATOR.leader.getPosition().getValueAsDouble(), Goal);
    SmartDashboard.putBoolean("workingiskhdjlj", true);

    SmartDashboard.putNumber("ele goals", controller.getGoal().position);
    SmartDashboard.putNumber("ksadjkfljasdlkfjlk", Goal);

    if (controller.atGoal()) {
      finishCommand = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("workingiskhdjlj", false);
    Constants.elevatorGoal = 0;
  }

  @Override
  public boolean isFinished() {
    return finishCommand;
  }
}
