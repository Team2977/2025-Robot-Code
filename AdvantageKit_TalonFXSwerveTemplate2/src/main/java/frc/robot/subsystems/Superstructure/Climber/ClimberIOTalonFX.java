// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.Climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO {

  @AutoLogOutput private final TalonFX falcon = new TalonFX(101, "rio");

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  @AutoLogOutput private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private double requestNumber = 0;

  public ClimberIOTalonFX() {
    // TODO MUST FIND LATER
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.MotionMagic.MotionMagicAcceleration = 2;
    config.MotionMagic.MotionMagicCruiseVelocity = 5;
    config.MotionMagic.MotionMagicJerk = 1;

    tryUntilOk(5, () -> falcon.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.connected = falcon.isConnected();
    inputs.positionRads = falcon.getPosition().getValueAsDouble();
    inputs.velocityRadsPerSec = falcon.getVelocity().getValueAsDouble();
    inputs.appliedVoltage = falcon.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = falcon.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = falcon.getTorqueCurrent().getValueAsDouble();
    inputs.tempCelsius = falcon.getDeviceTemp().getValueAsDouble();

    Logger.recordOutput(
        "Climber/falcon/motionMagicRequest",
        (DoubleSupplier) motionMagicRequest.withPosition(requestNumber));
  }

  @Override
  public void climberUp() {
    falcon.setControl(motionMagicRequest.withPosition(100));
    requestNumber = 100;
  }

  @Override
  public void climberDown() {
    falcon.setControl(motionMagicRequest.withPosition(0));
    requestNumber = 0;
  }

  @Override
  public void stopMotor() {
    falcon.stopMotor();
  }
}
