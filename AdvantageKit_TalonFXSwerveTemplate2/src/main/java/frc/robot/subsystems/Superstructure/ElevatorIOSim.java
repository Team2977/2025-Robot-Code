// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  // motors
  private final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);
  private final TalonFX falcon = new TalonFX(constantsE.kMotorPort);
  private final TalonFX falcon2 = new TalonFX(1);
  private final TalonFXSimState falconSim = new TalonFXSimState(falcon);
  private final TalonFXSimState falconSim2 = new TalonFXSimState(falcon2);
  // motor config
  private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  // encoder
  private final Encoder encoder =
      new Encoder(constantsE.kEncoderAChannel, constantsE.kEncoderBChannel);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  // colors for elevator simulator
  private final Color8Bit red = new Color8Bit(Color.kRed);
  private final Color8Bit green = new Color8Bit(Color.kGreen);

  // elevator pid stuff
  private double goal = 0;
  private double feedforwardOutput = 0;
  private double pidOutput = 0;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          constantsE.kElevatorKp,
          constantsE.kElevatorKi,
          constantsE.kElevatorKd,
          new TrapezoidProfile.Constraints(constantsE.kElevatorMaxVelocity, 6.6));
  ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          constantsE.kElevatorkS,
          constantsE.kElevatorkG,
          constantsE.kElevatorkV,
          constantsE.kElevatorkA);

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

  // Create a Mechanism2d visualization of the elevator
  private final LoggedMechanism2d mech2d =
      new LoggedMechanism2d(Units.inchesToMeters(20), Units.inchesToMeters(50), red);
  private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
  private final LoggedMechanismLigament2d elevatorMech2d =
      mech2dRoot.append(
          new LoggedMechanismLigament2d(
              "Elevator", elevatorSim.getPositionMeters(), 90, Units.inchesToMeters(20), green));

  /** Subsystem constructor. */
  public ElevatorIOSim() {
    encoder.setDistancePerPulse(constantsE.kElevatorEncoderDistPerPulse);
    encoderSim.setDistancePerPulse(constantsE.kElevatorEncoderDistPerPulse);
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> falcon.getConfigurator().apply(talonConfig));
    tryUntilOk(5, () -> falcon2.getConfigurator().apply(talonConfig));

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
    elevatorSim.setInput(
        (falconSim.getMotorVoltage() + falconSim2.getMotorVoltage())
            * RobotController.getBatteryVoltage());
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
    this.goal = goal;
    pidOutput = controller.calculate(encoder.getDistance(), goal);

    if (!controller.atGoal()) {
      feedforwardOutput = feedforward.calculate(controller.getSetpoint().velocity);
    } else {
      feedforwardOutput = 0;
    }

    falcon.set(pidOutput + feedforwardOutput);
    falcon2.set(pidOutput + feedforwardOutput);
    // logging data
    SmartDashboard.putNumber("feedforward", feedforwardOutput);
    Logger.recordOutput("feedforward", feedforwardOutput);
    Logger.recordOutput("pid output", pidOutput);
    Logger.recordOutput("PID Goal", goal);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    controller.setGoal(0.0);
    falcon.set(0.0);
    falcon2.set(0.0);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leaderConnected = falcon.isConnected();
    inputs.leaderPositionRota = falcon.getPosition().getValueAsDouble();
    inputs.leaderVelocityRotaPerSec = falcon.getVelocity().getValueAsDouble();
    inputs.leaderAppliedVoltage = falcon.getSupplyVoltage().getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = falcon.getSupplyCurrent().getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = falcon.getTorqueCurrent().getValueAsDouble();
    inputs.leaderTempCelsius = falcon.getDeviceTemp().getValueAsDouble();

    inputs.followerConnected = falcon2.isConnected();
    inputs.followerPositionRota = falcon2.getPosition().getValueAsDouble();
    inputs.followerVelocityRotaPerSec = falcon2.getVelocity().getValueAsDouble();
    inputs.followerAppliedVoltage = falcon2.getSupplyVoltage().getValueAsDouble();
    inputs.followerSupplyCurrentAmps = falcon2.getSupplyCurrent().getValueAsDouble();
    inputs.followerTorqueCurrentAmps = falcon2.getTorqueCurrent().getValueAsDouble();
    inputs.followerTempCelsius = falcon2.getDeviceTemp().getValueAsDouble();

    inputs.goal = goal;
    inputs.PIDControllerOutput = pidOutput;
    inputs.feedforwardOutput = feedforwardOutput;
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    elevatorMech2d.setLength(encoder.getDistance());
    SmartDashboard.putNumber("elvatorpose", encoder.getDistance());
    SmartDashboard.putNumber("elevator goal", controller.getGoal().position);
    Logger.recordOutput("eleSIM", mech2d);
    SmartDashboard.putNumber("vel", constantsE.kElevatorMaxVelocity);
  }
}
