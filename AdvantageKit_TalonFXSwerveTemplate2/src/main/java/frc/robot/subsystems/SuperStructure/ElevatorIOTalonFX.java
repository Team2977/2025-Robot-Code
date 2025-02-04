// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SuperStructure.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.other.MotorIO.MotorIOInputs;
import frc.robot.util.PhoenixUtil;
import frc.robot.subsystems.SuperStructure.ElevatorIO;

/** Add your docs here. */
public class ElevatorIOTalonFX {
     /** Creates a new MotorIOTalonFX. */
  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Temperature> leaderTempCelsius;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Temperature> followerTempCelsius;


  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final double reduction;

  public ElevatorIOTalonFX(
      int id, String bus, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    leader = new TalonFX(id, bus);
    follower = new TalonFX(id, bus);
    follower.setControl(new Follower(leader.getDeviceID(), false));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));
    

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderTorqueCurrent = leader.getTorqueCurrent();
    leaderTempCelsius = leader.getDeviceTemp();

    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerAppliedVoltage = follower.getMotorVoltage();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerTempCelsius = follower.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leaderPosition,
                leaderVelocity,
                leaderAppliedVoltage,
                leaderSupplyCurrent,
                leaderTorqueCurrent,
                leaderTempCelsius));
    PhoenixUtil.tryUntilOk(5, () -> leader.optimizeBusUtilization(0, 1.0));

    PhoenixUtil.tryUntilOk(5, 
        () -> 
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                followerPosition,
                followerVelocity,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent,
                followerTempCelsius
    ));
    PhoenixUtil.tryUntilOk(5, () -> follower.optimizeBusUtilization(0, 1.0));
  } 

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    //leader
    inputs.leaderConnected =
        BaseStatusSignal.refreshAll(
                leaderPosition, leaderVelocity, leaderAppliedVoltage, leaderSupplyCurrent, leaderTorqueCurrent, leaderTempCelsius)
            .isOK();
    //follower
    inputs.followerConnected =
        BaseStatusSignal.refreshAll(
                followerPosition, followerVelocity, followerAppliedVoltage, followerSupplyCurrent, followerTorqueCurrent, followerTempCelsius)
            .isOK();
    //leader
    inputs.leaderPositionRads = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / reduction;
    inputs.leaderVelocityRadsPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / reduction;
    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrent.getValueAsDouble();
    inputs.leaderTempCelsius = leaderTempCelsius.getValueAsDouble();

    // follower motor
    inputs.followerPositionRads = Units.rotationsToRadians(followerPosition.getValueAsDouble()) / reduction;
    inputs.followerVelocityRadsPerSec = Units.rotationsToRadians(followerVelocity.getValueAsDouble()) / reduction;
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVoltsElevator(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stopElevator() {
    leader.stopMotor();
  }
}
