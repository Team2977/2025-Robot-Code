// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperStructure.autoAim;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoAimCom extends Command {
  private final autoAim AUTOAIM;
  private final Drive drive;

  private static final ProfiledPIDController xController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)));

  private static final ProfiledPIDController yController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)));

  private static final ProfiledPIDController OmegaController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)));

  /** Creates a new autoAimCom. */
  public autoAimCom(autoAim AUTOAIM, Drive drive) {
    this.AUTOAIM = AUTOAIM;
    this.drive = drive;
    addRequirements(AUTOAIM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset(drive.getPose().getX());
    yController.reset(drive.getPose().getY());
    OmegaController.reset(drive.getPose().getRotation().getRadians());
    OmegaController.enableContinuousInput(0, 2 * Math.PI);

    Constants.autodrivingStuff.autoDrive = true;
    Constants.autodrivingStuff.xVal = 0;
    Constants.autodrivingStuff.yVal = 0;
    Constants.autodrivingStuff.omegaVal = 0;
    SmartDashboard.putBoolean("autoaimCom", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double currentElevatorPose = elevator.leader.getPosition().getValueAsDouble();
    Pose2d goal = autoAim.closestPose2d;

    Constants.autodrivingStuff.xVal = xController.calculate(drive.getPose().getX(), goal.getX());
    Constants.autodrivingStuff.yVal = yController.calculate(drive.getPose().getY(), goal.getY());
    Constants.autodrivingStuff.omegaVal =
        OmegaController.calculate(
            drive.getRotation().getRadians(), goal.getRotation().getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.autodrivingStuff.autoDrive = false;
    Constants.autodrivingStuff.xVal = 0;
    Constants.autodrivingStuff.yVal = 0;
    Constants.autodrivingStuff.omegaVal = 0;
    SmartDashboard.putBoolean("autoaimCom", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
