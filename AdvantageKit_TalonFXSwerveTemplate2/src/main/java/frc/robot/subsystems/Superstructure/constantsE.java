// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class constantsE {

  public static double GoalElevator = 0;

  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final double kElevatorKp = 10;
  public static final double kElevatorKi = 1;
  public static final double kElevatorKd = 0.1;

  public static final double kElevatorkS = 0.0; // volts (V)                    0.0
  public static final double kElevatorkG = 0.35; // volts (V)                  0.762
  public static final double kElevatorkV = 0.35; // volt per velocity (V/(m/s))    0.762
  public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

  public static final double kElevatorGearing = 45.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(0.935);
  public static final double kCarriageMass = 4.0; // kg

  public static final double kSetpointMeters = 0.75;
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final double kMinElevatorHeightMeters = 0.0;
  public static final double kMaxElevatorHeightMeters = 100;

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  public static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 2048;

  public static final double kElevatorMaxVelocity = kElevatorDrumRadius * (6380 / 60) * 2 * Math.PI;

  public class constantsTalonFX {

    public static final double currentLimitAmps = 40;
    public static final double gearReduction = 45.0;

    public static final double ElevatorkP = 0;
    public static final double ElevatorkI = 0;
    public static final double ElevatorkD = 0;

    public static final double ElevatorkS = 0.0; // volts (V)
    public static final double ElevatorkG = 0.0; // volts (V)
    public static final double ElevatorkV = 0.0; // volt per velocity (V/(m/s))
    public static final double ElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  }
}
