// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);

  private final Color8Bit red = new Color8Bit(Color.kRed);
  private final Color8Bit green = new Color8Bit(Color.kGreen);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          constantsE.kElevatorKp,
          constantsE.kElevatorKi,
          constantsE.kElevatorKd,
          new TrapezoidProfile.Constraints(constantsE.kElevatorMaxVelocity, 1000));
  ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          constantsE.kElevatorkS,
          constantsE.kElevatorkG,
          constantsE.kElevatorkV,
          constantsE.kElevatorkA);
  private final Encoder encoder =
      new Encoder(constantsE.kEncoderAChannel, constantsE.kEncoderBChannel);
  private final TalonFX falcon = new TalonFX(constantsE.kMotorPort);
  private final MotionMagicConfigs configsMotion = new MotionMagicConfigs();
  private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          constantsE.kElevatorGearing,
          constantsE.kCarriageMass,
          constantsE.kElevatorDrumRadius,
          constantsE.kMinElevatorHeightMeters,
          constantsE.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);
  private final EncoderSim encoderSim = new EncoderSim(encoder);
  private final TalonFXSimState falconSim = new TalonFXSimState(falcon);

  // Create a Mechanism2d visualization of the elevator
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(20, 50, red);
  private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
  private final LoggedMechanismLigament2d elevatorMech2d =
      mech2dRoot.append(
          new LoggedMechanismLigament2d(
              "Elevator", elevatorSim.getPositionMeters(), 90, 40, green));

  /** Subsystem constructor. */
  public ElevatorIOSim() {
    encoder.setDistancePerPulse(constantsE.kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);

    /*
    configsMotion.MotionMagicCruiseVelocity = constantsE.kElevatorMaxVelocity;
    configsMotion.MotionMagicAcceleration = constantsE.kElevatorMaxVelocity;
    configsMotion.MotionMagicExpo_kA = 0.5; // volts/rot^2
    configsMotion.MotionMagicExpo_kV = 0.5; // volts/rot
    configsMotion.MotionMagicJerk = 1;

    talonConfig.Slot0.kA = 0.5;
    talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    talonConfig.Slot0.kP = constantsE.kElevatorKp;
    talonConfig.Slot0.kI = constantsE.kElevatorKi;
    talonConfig.Slot0.kD = constantsE.kElevatorKd;
    talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    //talonConfig.Slot0.*/

  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    elevatorSim.setInput(falconSim.getMotorVoltage() * RobotController.getBatteryVoltage());
    elevatorSim.update(0.020);
    encoderSim.setDistance(elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    controller.setTolerance(Units.inchesToMeters(0.5));
    double pidOutput = controller.calculate(encoder.getDistance(), goal);
    double feedforwardOutput = feedforward.calculate(controller.getSetpoint().velocity);
    falcon.setVoltage(pidOutput + feedforwardOutput);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    controller.setGoal(0.0);
    falcon.set(0.0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    elevatorMech2d.setLength(encoder.getDistance());
    SmartDashboard.putNumber("elvatorpose", encoder.getDistance());
    SmartDashboard.putNumber("elevator goal", controller.getGoal().position);
    Logger.recordOutput("eleSIM", mech2d);
  }
}
