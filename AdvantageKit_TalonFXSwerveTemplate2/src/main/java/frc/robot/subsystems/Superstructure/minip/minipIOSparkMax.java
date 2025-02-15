// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.minip;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class MinipIOSparkMax implements MinipIO {
  private static final SparkMax ulator = new SparkMax(90, MotorType.kBrushless);
  private static final SparkMax ulatorSlave = new SparkMax(91, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();

  public MinipIOSparkMax() {
    // motor setup
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    // TODO check the breaker on the motors
    config.smartCurrentLimit(30);

    ulator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ulatorSlave.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(MinipIOInputs inputs) {
    // ulator stuff
    int faults = ulator.getFaults().rawBits;
    if (faults == 0) {
      inputs.ulatorConnected = true;
    } else {
      inputs.ulatorConnected = false;
    }

    inputs.ulatorPositionRota = ulator.getEncoder().getPosition();
    inputs.ulatorVelocityRPM = ulator.getEncoder().getVelocity();
    // TODO applided voltage can't be pulled strait from motor. must do later
    inputs.ulatorAppliedVoltage = 0;
    inputs.ulatorOutputAmps = ulator.getOutputCurrent();
    inputs.ulatorTempCelsius = ulator.getMotorTemperature();

    // ulatorSlave stuff
    int faults2 = ulatorSlave.getFaults().rawBits;
    if (faults2 == 0) {
      inputs.ulatorSlaveConnected = true;
    } else {
      inputs.ulatorSlaveConnected = false;
    }

    inputs.ulatorSlavePositionRota = ulatorSlave.getEncoder().getPosition();
    inputs.ulatorSlaveVelocityRPM = ulatorSlave.getEncoder().getVelocity();
    // TODO applided voltage can't be pulled strait from motor. must do later
    inputs.ulatorSlaveAppliedVoltage = 0;
    inputs.ulatorSlaveOutputApms = ulatorSlave.getOutputCurrent();
    inputs.ulatorSlaveTempCelsius = ulatorSlave.getMotorTemperature();

    Logger.recordOutput("minip/ulator/speed", ulator.get());
    Logger.recordOutput("minip/ulatorSlave/speed", ulatorSlave.get());
  }

  @Override
  public void minipIntake() {
    ulator.set(0.3);
    ulatorSlave.set(0.3);
  }

  @Override
  public void minipSpit() {
    ulator.set(-0.3);
    ulatorSlave.set(-0.3);
  }

  @Override
  public void stopMotors() {
    ulator.stopMotor();
    ulatorSlave.stopMotor();
  }
}
