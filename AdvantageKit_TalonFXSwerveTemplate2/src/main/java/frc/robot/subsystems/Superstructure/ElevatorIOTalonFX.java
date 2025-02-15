// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  // motors and other objects
  public final TalonFX leader = new TalonFX(1, "rio");
  public final TalonFX follower = new TalonFX(2, "rio");
  @AutoLogOutput private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private Follower slave = new Follower(leader.getDeviceID(), false);

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

    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
    config.HardwareLimitSwitch.ReverseLimitEnable = true;
    config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // TODO must find this. give some leaway.
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100;

    config.Feedback.RotorToSensorRatio = constantsE.constantsTalonFX.gearReduction;

    // TODO take a look at this
    // config.Feedback.SensorToMechanismRatio

    // TODO stop here for changes

    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    tryUntilOk(5, () -> follower.getConfigurator().apply(config));
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

    Logger.recordOutput("Elevator goal", Constants.elevatorGoal);
  }

  @Override
  public void REALrunToPosition(double goal) {
    this.goal = goal;
    pidOutput = controller.calculate(leader.getPosition().getValueAsDouble(), goal);
    feedforwardOutput = feedforward.calculate(controller.getSetpoint().velocity);

    // TODO make this work later

    leader.setControl(motionMagicRequest.withPosition(Constants.elevatorGoal));
    follower.setControl(slave);
  }
  /*
  @Override
  public inputState getState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getState'");
  }

  @Override
  public void setState(outputState output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setState'");
  }*/

  public void updateTelemetry() {}
}
