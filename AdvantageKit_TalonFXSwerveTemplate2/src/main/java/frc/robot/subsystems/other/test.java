// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.other;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class test extends SubsystemBase {

  public static final SparkMax motor = new SparkMax(0, MotorType.kBrushless);

  public test() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
