// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class autoBuilderSub extends SubsystemBase {
  private final LoggedDashboardChooser<String> startingPoint =
      new LoggedDashboardChooser<>("startingPoint");
  private final LoggedDashboardChooser<Integer> corralNumb =
      new LoggedDashboardChooser<>("corralNumb");
  private final LoggedDashboardChooser<Boolean> endQuick = new LoggedDashboardChooser<>("endQuick");
  /** Creates a new autoBuilderSub. */
  public autoBuilderSub() {
    startingPoint.addOption("Left", "left");
    startingPoint.addOption("Right", "right");
    startingPoint.addOption("Center", "center");

    corralNumb.addOption("1 corral", 1);
    corralNumb.addOption("2 corral", 2);
    corralNumb.addOption("3 corral", 3);
    corralNumb.addOption("4 corral", 4);

    endQuick.addOption("end quick true", true);
    endQuick.addOption("end quick false", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public static Command updatingPathCommand(Drive drive) {
    Pose2d curPose = drive.getPose();
    Pose2d goalPose = frc.robot.subsystems.SuperStructure.autoAim.rightSidePose2d;
    // Rotation2d pathRotation2d = frc.robot.subsystems.SuperStructure.autoAim.pathRotation2d;

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(curPose.getX(), curPose.getY(), Rotation2d.fromDegrees(0)),
            new Pose2d(goalPose.getX(), goalPose.getY(), Rotation2d.fromDegrees(0)));

    // The values are low so if anything goes wrong we can disable the robot
    // PathConstraints constraints = new PathConstraints(0.5, 1, 2 * Math.PI, 4 * Math.PI);
    PathConstraints constraints =
        new PathConstraints(4.18, 5, Units.degreesToRadians(500), Units.degreesToRadians(700));

    PathPlannerPath alignmentPath =
        new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0, goalPose.getRotation().plus(Rotation2d.kCCW_90deg)));

    // Logger.recordOutput("wantedPose", goalPose);

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return AutoBuilder.followPath(alignmentPath.flipPath());
    } else {
      return AutoBuilder.followPath(alignmentPath);
    }
  }
}
