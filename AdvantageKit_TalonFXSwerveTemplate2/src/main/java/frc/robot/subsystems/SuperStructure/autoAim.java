// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldPOIs;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class autoAim extends SubsystemBase {
  private AprilTagFieldLayout layout;
  public static Pose2d closestPose2d;
  public static Pose2d rightSidePose2d;
  // private Drive drive;

  private double tagDis1;
  private double tagDis2;
  private double tagDis3;
  private double tagDis4;
  private double tagDis5;
  private double tagDis6;

  // TODO enum with switch case
  public autoAim() {
    try {
      layout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
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

    int onRedTeam;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      onRedTeam = 1;
    } else {
      onRedTeam = 0;
    }

    switch (onRedTeam) {
      case 0: // RED
        tagDis1 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[0]);
        tagDis2 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[1]);
        tagDis3 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[2]);
        tagDis4 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[3]);
        tagDis5 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[4]);
        tagDis6 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(), FieldConstants.Reef.centerFaces[5]);
        break;

      case 1: // BLUE
        tagDis1 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(),
                AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[0]));
        tagDis2 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(),
                AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[1]));
        tagDis3 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(),
                AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[2]));
        tagDis4 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(),
                AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[3]));
        tagDis5 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(),
                AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[4]));
        tagDis6 =
            PhotonUtils.getDistanceToPose(
                RobotContainer.drive.getPose(),
                AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[5]));
        break;

      default:
        break;
    }

    double[] List = {tagDis1, tagDis2, tagDis3, tagDis4, tagDis5, tagDis6};

    int minIndex = 0; // Initialize with the first index
    for (int i = 1; i < List.length; i++) {
      if (List[i] < List[minIndex]) {
        minIndex = i; // Update the index of the smallest number
      }
    }
    SmartDashboard.putNumber("minIndex", minIndex);

    switch (onRedTeam) {
      case 1: // RED
        switch (minIndex) {
          case 0:
            closestPose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_LEFT.get(2));
            rightSidePose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_RIGHT.get(2));
            break;
          case 1:
            closestPose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_LEFT.get(1));
            rightSidePose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_RIGHT.get(1));
            break;
          case 2:
            closestPose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_LEFT.get(0));
            rightSidePose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_RIGHT.get(0));
            break;
          case 3:
            closestPose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_LEFT.get(5));
            rightSidePose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_RIGHT.get(5));
            break;
          case 4:
            closestPose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_LEFT.get(4));
            rightSidePose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_RIGHT.get(4));
            break;
          case 5:
            closestPose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_LEFT.get(3));
            rightSidePose2d = AllianceFlipUtil.apply(FieldPOIs.REEF_LOCATIONS_RIGHT.get(3));
            break;

          default:
            closestPose2d = new Pose2d();
            rightSidePose2d = new Pose2d();
            break;
        }
        break;
      case 0:
        switch (minIndex) {
          case 0:
            closestPose2d = FieldPOIs.REEF_LOCATIONS_LEFT.get(2);
            rightSidePose2d = FieldPOIs.REEF_LOCATIONS_RIGHT.get(2);
            break;
          case 1:
            closestPose2d = FieldPOIs.REEF_LOCATIONS_LEFT.get(1);
            rightSidePose2d = FieldPOIs.REEF_LOCATIONS_RIGHT.get(1);
            break;
          case 2:
            closestPose2d = FieldPOIs.REEF_LOCATIONS_LEFT.get(0);
            rightSidePose2d = FieldPOIs.REEF_LOCATIONS_RIGHT.get(0);
            break;
          case 3:
            closestPose2d = FieldPOIs.REEF_LOCATIONS_LEFT.get(5);
            rightSidePose2d = FieldPOIs.REEF_LOCATIONS_RIGHT.get(5);
            break;
          case 4:
            closestPose2d = FieldPOIs.REEF_LOCATIONS_LEFT.get(4);
            rightSidePose2d = FieldPOIs.REEF_LOCATIONS_RIGHT.get(4);
            break;
          case 5:
            closestPose2d = FieldPOIs.REEF_LOCATIONS_LEFT.get(3);
            rightSidePose2d = FieldPOIs.REEF_LOCATIONS_RIGHT.get(3);
            break;

          default:
            closestPose2d = new Pose2d();
            rightSidePose2d = new Pose2d();
            break;
        }
        break;

      default:
        break;
    }


    SmartDashboard.putNumber("wanted pose x", closestPose2d.getX());
    SmartDashboard.putNumber("wanted pose Y", closestPose2d.getY());
    SmartDashboard.putNumber("wanted pose rota", closestPose2d.getRotation().getDegrees());

    Logger.recordOutput(
        "wantedPose clostest",
        closestPose2d.rotateAround(closestPose2d.getTranslation(), Rotation2d.kCW_Pi_2));
    Logger.recordOutput(
        "wanted pose right",
        rightSidePose2d.rotateAround(rightSidePose2d.getTranslation(), Rotation2d.kCW_Pi_2));


  }
  /**
   * @param goalPose the target position in Pose2d form
   * @param drive Drive subsystem
   * @return target angle as Rotation2d from radians
   */

   /*
     public static Rotation2d targetAngle(Pose2d goalPose, Drive drive) {
       Pose2d curPose2d = drive.getPose();
       double deltaX = goalPose.getX() - curPose2d.getX();
       double deltaY = goalPose.getY() - curPose2d.getY();

       double targetAngle = Math.atan2(deltaY, deltaX);

       // SmartDashboard.putNumber("target angle", Units.radiansToDegrees(targetAngle));
       return Rotation2d.fromRadians(targetAngle);
     }
   }
    */
