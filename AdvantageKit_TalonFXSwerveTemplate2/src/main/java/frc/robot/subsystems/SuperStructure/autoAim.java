// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
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

    // layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // 6 - 11

    double tagDis1 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[0]);
    double tagDis2 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[1]);
    double tagDis3 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[2]);
    double tagDis4 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[3]);
    double tagDis5 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[4]);
    double tagDis6 =
        PhotonUtils.getDistanceToPose(
            RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[5]);

    double[] List = {tagDis1, tagDis2, tagDis3, tagDis4, tagDis5, tagDis6};

    int minIndex = 0; // Initialize with the first index
    for (int i = 1; i < List.length; i++) {
      if (List[i] < List[minIndex]) {
        minIndex = i; // Update the index of the smallest number
      }
    }
    /*
    switch (minIndex) {
      case 0:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(Constants.autodrivingStuff.tag1).get().toPose2d().getX(),
                    layout.getTagPose(Constants.autodrivingStuff.tag1).get().toPose2d().getY()),
                new Rotation2d(Units.degreesToRadians(-120)));
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 1:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(Constants.autodrivingStuff.tag2).get().toPose2d().getX(),
                    layout.getTagPose(Constants.autodrivingStuff.tag2).get().toPose2d().getY()),
                new Rotation2d(Units.degreesToRadians(180)));
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 2:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(Constants.autodrivingStuff.tag3).get().toPose2d().getX(),
                    layout.getTagPose(Constants.autodrivingStuff.tag3).get().toPose2d().getY()),
                new Rotation2d(Units.degreesToRadians(120)));
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 3:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(Constants.autodrivingStuff.tag4).get().toPose2d().getX(),
                    layout.getTagPose(Constants.autodrivingStuff.tag4).get().toPose2d().getY()),
                new Rotation2d(Units.degreesToRadians(60)));
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 4:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(Constants.autodrivingStuff.tag5).get().toPose2d().getX(),
                    layout.getTagPose(Constants.autodrivingStuff.tag5).get().toPose2d().getY()),
                new Rotation2d(0));
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 5:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    layout.getTagPose(Constants.autodrivingStuff.tag6).get().toPose2d().getX(),
                    layout.getTagPose(Constants.autodrivingStuff.tag6).get().toPose2d().getY()),
                new Rotation2d(Units.degreesToRadians(-60)));
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;

      default:
        closestPose2d = null;
        // SmartDashboard.putData("wanted pose", (Sendable) closestPose2d);
        break;
    }*/

    switch (minIndex) {
      case 0:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    FieldConstants.Reef.centerFaces[0].getX(),
                    FieldConstants.Reef.centerFaces[0].getY()),
                FieldConstants.Reef.centerFaces[0].getRotation());
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 1:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    FieldConstants.Reef.centerFaces[1].getX(),
                    FieldConstants.Reef.centerFaces[1].getY()),
                FieldConstants.Reef.centerFaces[1].getRotation());
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 2:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    FieldConstants.Reef.centerFaces[2].getX(),
                    FieldConstants.Reef.centerFaces[2].getY()),
                FieldConstants.Reef.centerFaces[2].getRotation());
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 3:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    FieldConstants.Reef.centerFaces[3].getX(),
                    FieldConstants.Reef.centerFaces[3].getY()),
                FieldConstants.Reef.centerFaces[3].getRotation());
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 4:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    FieldConstants.Reef.centerFaces[4].getX(),
                    FieldConstants.Reef.centerFaces[4].getY()),
                FieldConstants.Reef.centerFaces[4].getRotation());
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;
      case 5:
        closestPose2d =
            new Pose2d(
                new Translation2d(
                    FieldConstants.Reef.centerFaces[5].getX(),
                    FieldConstants.Reef.centerFaces[5].getY()),
                FieldConstants.Reef.centerFaces[5].getRotation());
        SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
        SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
        SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());
        break;

      default:
        closestPose2d = null;
        // SmartDashboard.putData("wanted pose", (Sendable) closestPose2d);
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
