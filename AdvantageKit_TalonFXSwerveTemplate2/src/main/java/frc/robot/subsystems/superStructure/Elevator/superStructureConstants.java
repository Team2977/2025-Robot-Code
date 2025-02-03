// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superStructure.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class superStructureConstants {
  // 2d position of superstructure root on robot (x forward from back, y off the bellypan)
  public static final Translation2d superstructureOrigin2d = new Translation2d(0.10686, 0.06357);
  public static final Translation3d superstructureOrigin3d =
      new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());
  public static final double pivotLength = 0.16;
  public static final double pivotLengthBack = 0.05;
  public static final double pivotToGripper = 0.207;
  public static final double G = 9.807;
  // From inside face to inside face
  public static final double stageHeight = 0.808; // Measured from CAD
  public static final double stageThickness = 0.025;
  public static final double carriageToStage = 0.12;
  public static final double stageToStage = 0.127;
  public static final double elevatorHeightMeters = 1.64;
  public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(82.0);
  public static final Translation3d carriageOrigin3d =
      superstructureOrigin3d.plus(
          new Translation3d(
              stageThickness * 2 + carriageToStage,
              new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0)));
  public static final Rotation2d pivotSafeAngle = Rotation2d.fromDegrees(-40.0);
  public static final LoggedTunableNumber throwHeight =
      new LoggedTunableNumber("Superstructure/Throw/Height", 1.75);
  public static final LoggedTunableNumber throwVelocity =
      new LoggedTunableNumber(
          "Superstructure/Throw/Velocity", 2.99999); // Exponential profile is weird
}
