// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
  private static DCMotorSim sim;
  private static DCMotorSim followerSim;
  private double appliedVoltage = 0.0;
  private Mechanism2d elevator = new Mechanism2d(20/*inches */, 100 /* inches */);
  private MechanismRoot2d root = elevator.getRoot("elevator", 2, 0);
  private MechanismLigament2d stage2 = new MechanismLigament2d("stage2", 100, 90);


  private static final DCMotor motorModel = DCMotor.getFalcon500(2);
  private static final double reduction = 45 /*(18.0 / 12.0)*/;
  private static final double moi = 0.001;

  public ElevatorIOSim(DCMotor motorModel, double reduction, double moi) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);

    followerSim =  new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    
  }
    


  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVoltsElevator(0.0);
    }

    inputs.leaderConnected = true;
    sim.update(Constants.loopPeriodSecs);
    inputs.leaderPositionRads = sim.getAngularPositionRad();
    inputs.leaderVelocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.leaderAppliedVoltage = appliedVoltage;
    inputs.leaderSupplyCurrentAmps = sim.getCurrentDrawAmps();

    inputs.followerConnected = true;
    followerSim.update(Constants.loopPeriodSecs);
    inputs.followerPositionRads = followerSim.getAngularPositionRad();
    inputs.followerVelocityRadsPerSec = followerSim.getAngularVelocityRadPerSec();
    inputs.followerAppliedVoltage = appliedVoltage;
    inputs.followerSupplyCurrentAmps = followerSim.getCurrentDrawAmps();
    
    SmartDashboard.putData("elevator", elevator);

  }

  @Override
  public void runVoltsElevator(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
    followerSim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stopElevator() {
    runVoltsElevator(0.0);
  }
}
