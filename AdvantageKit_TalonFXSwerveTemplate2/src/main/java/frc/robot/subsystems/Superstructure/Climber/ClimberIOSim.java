// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.Climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

  private final TalonFX falcon = new TalonFX(101, "rio");
  private final DCMotor falconSim = DCMotor.getFalcon500(1);
  private double appliedVoltage = 0;

  public ClimberIOSim() {
    // inoitalizer
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.connected = true;
    inputs.positionRads = falcon.getPosition().getValueAsDouble();
    inputs.velocityRadsPerSec = falcon.getVelocity().getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = falcon.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = falcon.getTorqueCurrent().getValueAsDouble();
    inputs.tempCelsius = falcon.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void climberUp() {
    appliedVoltage = 6;
    falcon.setVoltage(appliedVoltage);
  }

  @Override
  public void climberDown() {
    appliedVoltage = -6;
    falcon.setVoltage(appliedVoltage);
  }

  @Override
  public void stopMotor() {
    appliedVoltage = 0;
    falcon.stopMotor();
  }
}
