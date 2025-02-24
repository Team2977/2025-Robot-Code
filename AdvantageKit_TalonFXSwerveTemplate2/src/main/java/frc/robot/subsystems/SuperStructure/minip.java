// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class minip extends SubsystemBase {
  private static final SparkMax ulator = new SparkMax(3, MotorType.kBrushless);
  private static final SparkMax ulatorSlave = new SparkMax(4, MotorType.kBrushless);
  private final SparkMaxConfig configUlator = new SparkMaxConfig();
  private final SparkMaxConfig configSlave = new SparkMaxConfig();

  /** Creates a new minip. */
  public minip() {

    configUlator.idleMode(IdleMode.kBrake);
    configUlator.inverted(true);
    // TODO check the breaker on the motors
    configUlator.smartCurrentLimit(30);

    configSlave.idleMode(IdleMode.kBrake);
    configSlave.inverted(false);
    // TODO check the breaker on the motors
    configSlave.smartCurrentLimit(30);

    ulator.configure(configUlator, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ulatorSlave.configure(
        configSlave, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void minipIntake() {
    ulator.set(0.4);
    ulatorSlave.set(0.4);
  }

  public void minipSpit() {
    ulator.set(-0.6);
    ulatorSlave.set(-0.6);
  }

  public void stopMotors() {
    ulator.stopMotor();
    ulatorSlave.stopMotor();
  }
}
