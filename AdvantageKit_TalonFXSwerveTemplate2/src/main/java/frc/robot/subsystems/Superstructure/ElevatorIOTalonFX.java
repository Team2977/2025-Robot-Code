// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  // motors and other objects
  public final TalonFX leader = new TalonFX(1, "rio");
  public final TalonFX follower = new TalonFX(2, "rio");
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  // elevator pid values
  private double goal = 0;
  private double pidOutput = 0;
  private double feedforwardOutput = 0;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          constantsE.constantsTalonFX.ElevatorkP,
          constantsE.constantsTalonFX.ElevatorkI,
          constantsE.constantsTalonFX.ElevatorkD,
          new TrapezoidProfile.Constraints(constantsE.kElevatorMaxVelocity, 6.6));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          constantsE.constantsTalonFX.ElevatorkS,
          constantsE.constantsTalonFX.ElevatorkG,
          constantsE.constantsTalonFX.ElevatorkV,
          constantsE.constantsTalonFX.ElevatorkA);

  /* TODO see if this is nessesary for the robot
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
  private final StatusSignal<Temperature> followerTempCelsius;*/

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

    config.Feedback.RotorToSensorRatio = constantsE.constantsTalonFX.gearReduction;

    // TODO stop here for changes

    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    tryUntilOk(5, () -> follower.getConfigurator().apply(config));

    /* TODO see if this is nessesary for the robot
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
    followerTempCelsius = follower.getDeviceTemp();*/

  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leaderConnected = leader.isConnected();
    inputs.leaderPositionRota = leader.getPosition().getValueAsDouble();
    inputs.leaderVelocityRotaPerSec = leader.getVelocity().getValueAsDouble();
    inputs.leaderAppliedVoltage = leader.getSupplyVoltage().getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leader.getTorqueCurrent().getValueAsDouble();
    inputs.leaderTempCelsius = leader.getDeviceTemp().getValueAsDouble();

    inputs.followerConnected = follower.isConnected();
    inputs.followerPositionRota = follower.getPosition().getValueAsDouble();
    inputs.followerVelocityRotaPerSec = follower.getVelocity().getValueAsDouble();
    inputs.followerAppliedVoltage = follower.getSupplyVoltage().getValueAsDouble();
    inputs.followerSupplyCurrentAmps = follower.getSupplyCurrent().getValueAsDouble();
    inputs.followerTorqueCurrentAmps = follower.getTorqueCurrent().getValueAsDouble();
    inputs.followerTempCelsius = follower.getDeviceTemp().getValueAsDouble();

    inputs.goal = goal;
    inputs.PIDControllerOutput = pidOutput;
    inputs.feedforwardOutput = feedforwardOutput;
  }

  @Override
  public void REALrunToPosition(double goal) {
    this.goal = goal;
    pidOutput = controller.calculate(leader.getPosition().getValueAsDouble(), goal);
    feedforwardOutput = feedforward.calculate(controller.getSetpoint().velocity);

    leader.setControl(motionMagicRequest.withPosition(pidOutput + feedforwardOutput));
  }
}
