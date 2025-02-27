// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.elevator;
import frc.robot.subsystems.SuperStructure.minip;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoScoreL4 extends SequentialCommandGroup {
  /** Creates a new autoScoreL4. */
  public autoScoreL4(elevator elevator, minip minip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new moveElevator(elevator, Constants.reefLevels.L4).withTimeout(3),
        new minipOut(minip).withTimeout(2),
        new moveElevator(elevator, Constants.reefLevels.L1).withTimeout(3));
  }
}
