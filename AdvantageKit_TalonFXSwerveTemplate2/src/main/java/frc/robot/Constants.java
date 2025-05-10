// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static int teleopInvert = 1;

  public static class elevatorConstants {
    public static final double kp = 0.25;
    public static final double ki = 0.05;
    public static final double kd = 0;
    public static final double maxVel = 100;
    public static final double maxAccel = 200;
  }

  public static class automaticAlingment {
    public static final Pose2d feederFarLeft =
        new Pose2d(new Translation2d(1.54, 7.36), Rotation2d.fromDegrees(-144));
    public static final Pose2d feederFarRight =
        new Pose2d(new Translation2d(1.63, 0.64), Rotation2d.fromDegrees(144));

    public static final Pose2d feederNearLeft =
        new Pose2d(new Translation2d(0.74, 6.81), Rotation2d.fromDegrees(-144));
    public static final Pose2d feederNearRight =
        new Pose2d(new Translation2d(0.67, 1.31), Rotation2d.fromDegrees(144));
  }

  public static class autodrivingStuff {

    public static boolean autoDrive = false;
    public static double xVal = 0;
    public static double yVal = 0;
    public static double omegaVal = 0;

    public static int tag1 = 6;
    public static int tag2 = 7;
    public static int tag3 = 8;
    public static int tag4 = 9;
    public static int tag5 = 10;
    public static int tag6 = 11;
  }

  public static class reefLevels {
    public static final double L1 = 0;
    public static final double L2 = 54;
    public static final double L3 = 115;
    public static final double L4 = 228;
  }

  public static double elevatorGoal = 0;
  public static final double loopPeriodSecs = 0.02;
  public static int invert = 1;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
