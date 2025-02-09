// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  // motors and other objects
  public final TalonFX leader = new TalonFX(1, "rio");
  public final TalonFX follower = new TalonFX(2, "rio");

  //

  // leader
  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Temperature> leaderTempCelsius;
  // follower
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Temperature> followerTempCelsius;

  public ElevatorIOTalonFX() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = constantsE.constantsTalonFX.currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // TODO must find the values all these things
    config.MotionMagic.MotionMagicAcceleration = 0;
    config.MotionMagic.MotionMagicCruiseVelocity = 0;
    config.MotionMagic.MotionMagicExpo_kA = 0;
    config.MotionMagic.MotionMagicExpo_kV = 0;
    config.MotionMagic.MotionMagicJerk = 0;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    config.Slot0.kA = 0; // acceleration gains
    config.Slot0.kG = 0; // gravity feedforward gains
    config.Slot0.kS = 0; // static feedforward gains
    config.Slot0.kV = 0; // velocity feedforward gains

    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;
    config.HardwareLimitSwitch.ForwardLimitEnable = true;
    config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;

    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    tryUntilOk(5, () -> follower.getConfigurator().apply(config));

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
  }
}
