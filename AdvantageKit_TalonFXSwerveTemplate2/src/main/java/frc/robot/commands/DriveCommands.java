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

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.other.Motor;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonUtils;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  public static AprilTagFieldLayout layout;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public static Command TEMPLATE(Drive drive, double someDouble, boolean someBoolean) {

    return Commands.run(() -> {}, drive);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
  // #################################### JOYSTICK DRIVE AT ANGLE
  // ###################################################################################
  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  // ####################################### ALIGHN TO POSE AND ANGLE
  // #############################################################
  public static Command alignToPoseAndAngleCom(
      Drive drive, Supplier<Translation2d> poseSupplier, Supplier<Rotation2d> rotationSupplier) {

    double kp = 10;
    double ki = 1;
    double kd = 0;
    // create PID controller for X direction
    ProfiledPIDController xController =
        new ProfiledPIDController(
            kp, ki, kd, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, 20));
    xController.enableContinuousInput(-Math.PI, Math.PI);

    // create PID controller for Y direction
    ProfiledPIDController yController =
        new ProfiledPIDController(
            kp, ki, kd, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, 20));
    yController.enableContinuousInput(-Math.PI, Math.PI);

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                      rotationSupplier.get().getRadians());

              // calcualate linear velocity
              double XVel =
                  xController.calculate(
                      drive.poseEstimator.getEstimatedPosition().getX(), poseSupplier.get().getX());

              double YVel =
                  yController.calculate(
                      drive.poseEstimator.getEstimatedPosition().getY(), poseSupplier.get().getY());

              SmartDashboard.putNumber("xController", XVel);
              SmartDashboard.putNumber("yController", YVel);
              SmartDashboard.putNumber("omegaController", omega);
              SmartDashboard.putNumber("x Pose", drive.poseEstimator.getEstimatedPosition().getX());
              SmartDashboard.putNumber("y Pose", drive.poseEstimator.getEstimatedPosition().getY());

              Translation2d linearVelocity = getLinearVelocityFromJoysticks(-XVel, -YVel);
              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controllers when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .beforeStarting(() -> xController.reset(drive.getPose().getX()))
        .beforeStarting(() -> yController.reset(drive.getPose().getY()));
  }

  // ##################################### ALIGN TO FEEDER
  // ############################################################
  public static Command alignToFeederCom(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    double kp = 2;
    double ki = 0;
    double kd = 1;
    // create PID controller for X direction
    ProfiledPIDController xController =
        new ProfiledPIDController(
            kp, ki, kd, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, 20));
    xController.enableContinuousInput(-Math.PI, Math.PI);

    // create PID controller for Y direction
    ProfiledPIDController yController =
        new ProfiledPIDController(
            kp, ki, kd, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, 20));
    yController.enableContinuousInput(-Math.PI, Math.PI);

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              double XGoalRight = 1.15;
              double YGoalRight = 1.17;
              double XGoalLeft = 1.18;
              double YGoalLeft = 6.94;

              // if statement to check if the robot is nearer than 4 meters from the feeder
              if (drive.poseEstimator.getEstimatedPosition().getX() <= 4) {

                var rightFeederdis =
                    PhotonUtils.getDistanceToPose(
                        drive.poseEstimator.getEstimatedPosition(),
                        /*layout.getTagPose(2).get().toPose2d()*/ new Pose2d(
                            new Translation2d(XGoalRight, YGoalRight), new Rotation2d()));

                var leftFeederdis =
                    PhotonUtils.getDistanceToPose(
                        drive.poseEstimator.getEstimatedPosition(),
                        /*layout.getTagPose(1).get().toPose2d()*/ new Pose2d(
                            new Translation2d(XGoalLeft, YGoalLeft), new Rotation2d()));

                @SuppressWarnings("unused")
                Pose2d pose = new Pose2d();
                double XPoseVal = 0;
                double YPoseVal = 0;
                double angle = 0;
                double XVel = 0;
                double YVel = 0;
                double omega = 0;
                if (rightFeederdis < leftFeederdis
                    && drive.poseEstimator.getEstimatedPosition().getX() <= 4) {
                  // goes to right feeder station
                  pose =
                      new Pose2d(
                          new Translation2d(XGoalRight, YGoalRight),
                          new Rotation2d(Math.toRadians(-126)));
                  XPoseVal = XGoalRight;
                  YPoseVal = YGoalRight;
                  angle = -126;

                  // Calculate angular speed
                  omega =
                      angleController.calculate(
                          drive.poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                          Math.toRadians(angle));

                  // calcualate linear velocity
                  XVel =
                      xController.calculate(
                          drive.poseEstimator.getEstimatedPosition().getX(), XPoseVal);

                  YVel =
                      yController.calculate(
                          drive.poseEstimator.getEstimatedPosition().getY(), YPoseVal);

                } else if (drive.poseEstimator.getEstimatedPosition().getX() <= 4) {
                  // goes to left feeder station
                  pose =
                      new Pose2d(
                          new Translation2d(XGoalLeft, YGoalLeft),
                          new Rotation2d(Math.toRadians(126)));
                  XPoseVal = XGoalLeft;
                  YPoseVal = YGoalLeft;
                  angle = 126;
                  // Calculate angular speed
                  omega =
                      angleController.calculate(
                          drive.poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                          Math.toRadians(126));

                  // calcualate linear velocity
                  XVel =
                      xController.calculate(
                          drive.poseEstimator.getEstimatedPosition().getX(), XPoseVal);

                  YVel =
                      yController.calculate(
                          drive.poseEstimator.getEstimatedPosition().getY(), YPoseVal);
                }

                // numbers outputed to smartdashboard for debugging
                SmartDashboard.putNumber("xController", XVel);
                SmartDashboard.putNumber("yController", YVel);
                SmartDashboard.putNumber("omegaController", omega);
                SmartDashboard.putNumber(
                    "x Pose", drive.poseEstimator.getEstimatedPosition().getX());
                SmartDashboard.putNumber(
                    "y Pose", drive.poseEstimator.getEstimatedPosition().getY());
                SmartDashboard.putNumber("left distance", leftFeederdis);
                SmartDashboard.putNumber("rightfeeder dis", rightFeederdis);

                Translation2d linearVelocity = getLinearVelocityFromJoysticks(XVel, YVel);
                // Convert to field relative speeds & send command
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        -linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        -linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
                // else for when it is father than that
              } else {

                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(), ySupplier.getAsDouble());

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        -linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        -linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec());
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
              }
            },
            drive)

        // Reset PID controllers when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .beforeStarting(() -> xController.reset(drive.getPose().getX()))
        .beforeStarting(() -> yController.reset(drive.getPose().getY()));
  }

  public static Command moveMotorTestCom(Motor motor) {

    return Commands.run(
            () -> {
              motor.runMotor(4);
              SmartDashboard.putBoolean("comand on", true);
            },
            motor)
        .finallyDo(() -> SmartDashboard.putBoolean("comand on", false));
  }

  // ###################################### FEEDFORWARD
  // ##############################################################
  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
