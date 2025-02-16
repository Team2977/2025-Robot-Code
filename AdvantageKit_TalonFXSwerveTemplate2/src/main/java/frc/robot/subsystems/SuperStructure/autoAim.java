// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.io.IOException;
import org.photonvision.PhotonUtils;

public class autoAim extends SubsystemBase {
  private elevator ELEVATOR = new elevator();
  // private RobotContainer robotContainer = new RobotContainer();
  // private final Drive drive = RobotContainer.drive;
  private AprilTagFieldLayout layout;
  public static Pose2d closestPose2d;

  enum reefLevel {
    L1,
    L2,
    L3,
    L4
  }

  enum reefSide {
    right,
    left
  }

  // TODO enum with switch case
  public autoAim() {
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // 6 - 11

    double tagDis6 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), layout.getTagPose(6).get().toPose2d());
    double tagDis7 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), layout.getTagPose(7).get().toPose2d());
    double tagDis8 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), layout.getTagPose(8).get().toPose2d());
    double tagDis9 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), layout.getTagPose(9).get().toPose2d());
    double tagDis10 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), layout.getTagPose(10).get().toPose2d());
    double tagDis11 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), layout.getTagPose(11).get().toPose2d());
    double[] List = {tagDis6, tagDis7, tagDis8, tagDis9, tagDis10, tagDis11};

    int minIndex = 0; // Initialize with the first index
    for (int i = 1; i < List.length; i++) {
      if (List[i] < List[minIndex]) {
        minIndex = i; // Update the index of the smallest number
      }
    }

    switch (minIndex) {
      case 0:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(6).get().toPose2d().getX(),
                    layout.getTagPose(6).get().toPose2d().getY()),
                new Rotation2d(-120));
        break;
      case 1:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(7).get().toPose2d().getX(),
                    layout.getTagPose(7).get().toPose2d().getY()),
                new Rotation2d(180));
        break;
      case 2:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(8).get().toPose2d().getX(),
                    layout.getTagPose(8).get().toPose2d().getY()),
                new Rotation2d(120));
        break;
      case 3:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(9).get().toPose2d().getX(),
                    layout.getTagPose(9).get().toPose2d().getY()),
                new Rotation2d(60));
        break;
      case 4:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(10).get().toPose2d().getX(),
                    layout.getTagPose(10).get().toPose2d().getY()),
                new Rotation2d(0));
        break;
      case 5:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(6).get().toPose2d().getX(),
                    layout.getTagPose(6).get().toPose2d().getY()),
                new Rotation2d(-60));
        break;

      default:
        closestPose2d = null;
        break;
    }
  }

  /**
   * @param side what side of the reef. Acepts "right" and "left"
   * @param level what level. Acepts "L1", "L2", "L3", "L4"
   * @return returns double array with elevator hight,
   */
  public static double[] createElevatorCmd(reefSide side, reefLevel level) {

    return null;
  }
}
