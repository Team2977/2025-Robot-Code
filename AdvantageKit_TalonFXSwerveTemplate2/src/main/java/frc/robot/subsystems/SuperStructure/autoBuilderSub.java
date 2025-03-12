// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class autoBuilderSub extends SubsystemBase {
  private final LoggedDashboardChooser<String> startingPoint =
      new LoggedDashboardChooser<>("startingPoint");
  private final LoggedDashboardChooser<Integer> corralNumb =
      new LoggedDashboardChooser<>("corralNumb");
  private final LoggedDashboardChooser<Boolean> endQuick = new LoggedDashboardChooser<>("endQuick");
  /** Creates a new autoBuilderSub. */
  public autoBuilderSub() {
    startingPoint.addOption("Left", "left");
    startingPoint.addOption("Right", "right");
    startingPoint.addOption("Center", "center");

    corralNumb.addOption("1 corral", 1);
    corralNumb.addOption("2 corral", 2);
    corralNumb.addOption("3 corral", 3);
    corralNumb.addOption("4 corral", 4);

    endQuick.addOption("end quick true", true);
    endQuick.addOption("end quick false", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
