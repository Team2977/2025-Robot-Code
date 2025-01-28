// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.other;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class MotorIOSim implements MotorIO {
  private static DCMotorSim sim;
  private double appliedVoltage = 0.0;

  private static final DCMotor motorModel = DCMotor.getFalcon500(1);
  private static final double reduction = 0.0 /*(18.0 / 12.0)*/;
  private static final double moi = 0.001;

  public MotorIOSim(DCMotor motorModel, double reduction, double moi) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
