// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.other;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class test extends SubsystemBase {

  public static final TalonFX elevator1 = new TalonFX(1, "rio");
  public static final TalonFX elevator2 = new TalonFX(2, "rio");

  public test() {

    elevator1.setInverted(false);
    elevator2.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double speed = RobotContainer.opperator.getRawAxis(1) / 4;

    elevator1.set(speed);
    elevator2.set(speed);
  }
}
