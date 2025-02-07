// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SuperStructure.ElevatorState.inputState;
import frc.robot.subsystems.SuperStructure.ElevatorState.outputState;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
  // private static DCMotorSim sim;
  // private static DCMotorSim followerSim;

  private final PIDController pidController = new PIDController(0.0, 0.0, 0.0);
  private double appliedVoltage = 0.0;
  private double reduction = 45;

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          reduction,
          ElevatorConstants.carriageMassKg,
          ElevatorConstants.drumRatiusMeters,
          ElevatorConstants.minHightMeters,
          ElevatorConstants.maxHightMeters,
          true,
          ElevatorConstants.minHightMeters,
          0,
          0.1);

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVoltsElevator(0.0);
    }

    inputs.leaderConnected = true;
    // sim.update(Constants.loopPeriodSecs);
    // inputs.leaderPositionRads = sim.getAngularPositionRad();
    // inputs.leaderVelocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.leaderAppliedVoltage = appliedVoltage;
    // inputs.leaderSupplyCurrentAmps = sim.getCurrentDrawAmps();

    inputs.followerConnected = true;
    // followerSim.update(Constants.loopPeriodSecs);
    // inputs.followerPositionRads = followerSim.getAngularPositionRad();
    // inputs.followerVelocityRadsPerSec = followerSim.getAngularVelocityRadPerSec();
    inputs.followerAppliedVoltage = appliedVoltage;
    // inputs.followerSupplyCurrentAmps = followerSim.getCurrentDrawAmps();
  }

  @Override
  public void runVoltsElevator(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    // TODO see how to control the simulation from get/set states

  }

  @Override
  public void stopElevator() {
    runVoltsElevator(0.0);
  }

  @Override
  public void moveToPosition(double goal) {
    pidController.setTolerance(0.10);
    // runVoltsElevator(pidController.calculate(sim.getAngularPositionRad(), goal));
    SmartDashboard.putBoolean("on", true);
  }

  @Override
  public inputState getState() {
    elevatorSim.update(0.02);
    return new inputState(
        elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
  }

  @Override
  public void setState(outputState output) {
    output
        .voltage()
        .ifPresent(
            (appliedVoltage) -> {
              elevatorSim.setInputVoltage(appliedVoltage);
            });
  }
}
