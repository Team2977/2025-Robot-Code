// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.autoAim;
import frc.robot.subsystems.SuperStructure.elevator;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

/** Add your docs here. */
/*
public class autoGenerator {

  // ################################################ alignn to reef left side
  /**
   * AUTO align to reef left
   *
   * @param drive Drive subsystem
   * @param autoAim AutoAim subsystem
   * @return pathplanner command to execute for left side
   */

  /*
  public static Command autoReefLeft(Drive drive, autoAim autoAim) {
    Pose2d curPose = drive.getPose();
    Pose2d goalPose = frc.robot.subsystems.SuperStructure.autoAim.closestPose2d;

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
            new GoalEndState(0, goalPose.getRotation().plus(Rotation2d.kCW_90deg)));

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return AutoBuilder.followPath(alignmentPath.flipPath());
    } else {
      return AutoBuilder.followPath(alignmentPath);
    }

    // return AutoBuilder.followPath(alignmentPath);
  }

  // ################################################ alignn to reef Right side
  /**
   * AUTO align to reef right
   *
   * @param drive Drive subsystem
   * @param autoAim AutoAim subsystem
   * @return pathplanner command to execute for right side
   */

   /*
   public static Command autoReefRight(Drive drive, autoAim autoAim) {
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
             new GoalEndState(0, goalPose.getRotation().plus(Rotation2d.kCW_90deg)));

     if (DriverStation.getAlliance().get() == Alliance.Red) {
       return AutoBuilder.followPath(alignmentPath.flipPath());
     } else {
       return AutoBuilder.followPath(alignmentPath);
     }
   }

   /**
    * The diffrence between this and moveElevator.java is this has a timeout for auto. Currently
    * timeout is at 3 seconds
    *
    * @param elevator Elevator subsystem
    * @param goal The wanted goal of the elevator in rotaions of the motor
    * @return Command to run the elevator in auto
    */

   /*
     public static Command autoMoveElevator(elevator elevator, double goal) {
       final ProfiledPIDController controller =
           new ProfiledPIDController(
               Constants.elevatorConstants.kp,
               Constants.elevatorConstants.ki,
               Constants.elevatorConstants.kd,
               new TrapezoidProfile.Constraints(
                   Constants.elevatorConstants.maxVel, Constants.elevatorConstants.maxAccel));

       return Commands.run(
               () -> {
                 Constants.elevatorGoal =
                     controller.calculate(elevator.leader.getPosition().getValueAsDouble(), goal);
                 SmartDashboard.putBoolean("mvoing", true);
               },
               elevator)
           .beforeStarting(() -> controller.reset(elevator.leader.getPosition().getValueAsDouble()))
           .finallyDo(() -> controller.atGoal())
           .finallyDo(() -> SmartDashboard.putBoolean("mvoing", true))
           .withTimeout(3);
     }
   }
   */
