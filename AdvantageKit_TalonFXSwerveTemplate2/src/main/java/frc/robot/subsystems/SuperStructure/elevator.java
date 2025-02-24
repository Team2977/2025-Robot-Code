// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class elevator extends SubsystemBase {
  /** Creates a new elevator. */
  public static final TalonFX leader = new TalonFX(6, "rio");

  private static final TalonFX follower = new TalonFX(7, "rio");

  @AutoLogOutput
  private static final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  // motion magic does math in the background, runs on motor

  private Follower slave = new Follower(leader.getDeviceID(), true);

  public elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration = 0;
    config.MotionMagic.MotionMagicCruiseVelocity = 0;
    config.MotionMagic.MotionMagicExpo_kA = 0;
    config.MotionMagic.MotionMagicExpo_kV = 0;
    config.MotionMagic.MotionMagicJerk = 0;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    config.Slot0.kA = 0;
    config.Slot0.kG = 0;
    config.Slot0.kS = 0;
    config.Slot0.kV = 0;

    config.Slot0.kP = 10;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    config.Feedback.RotorToSensorRatio = 45;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // leader.setControl(motionMagicRequest.withPosition(Constants.elevatorGoal));
    leader.set(Constants.elevatorGoal);
    follower.setControl(slave);

    Logger.recordOutput("Elevtor Goal", Constants.elevatorGoal);
    Logger.recordOutput("Position Of Leader", leader.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("LeaderPosition Data", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Goal", Constants.elevatorGoal);
  }

  /*public void runToPosition(double Goal) {

    leader.setControl(motionMagicRequest.withPosition(Goal));
     follower.setControl(slave);

     SmartDashboard.putNumber("Goal", Goal);
  }
  */
}
