// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure.minip;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class MinipIOSim implements MinipIO {
  private final SparkMax ulator = new SparkMax(90, MotorType.kBrushless);
  private final SparkMax ulatorSlave = new SparkMax(91, MotorType.kBrushless);
  private final DCMotor ulatorDcMotor = DCMotor.getNeo550(1);
  private final SparkMaxSim ulatorSim = new SparkMaxSim(ulator, ulatorDcMotor);
  private final SparkMaxSim ulatorSlaveSim = new SparkMaxSim(ulatorSlave, ulatorDcMotor);

  private final SparkBaseConfig config = new SparkFlexConfig();

  public MinipIOSim() {
    // initalize
  }

  @Override
  public void updateInputs(MinipIOInputs inputs) {
    // ulator stuff
    int faults = ulator.getFaults().rawBits;
    if (faults == 0) {
      inputs.ulatorConnected = true;
    } else {
      inputs.ulatorConnected = false;
    }

    inputs.ulatorPositionRota = ulator.getEncoder().getPosition();
    inputs.ulatorVelocityRPM = ulator.getEncoder().getVelocity();
    // TODO applided voltage can't be pulled strait from motor. must do later
    inputs.ulatorAppliedVoltage = 0;
    inputs.ulatorOutputAmps = ulator.getOutputCurrent();
    inputs.ulatorTempCelsius = ulator.getMotorTemperature();

    // ulatorSlave stuff
    int faults2 = ulatorSlave.getFaults().rawBits;
    if (faults2 == 0) {
      inputs.ulatorSlaveConnected = true;
    } else {
      inputs.ulatorSlaveConnected = false;
    }

    inputs.ulatorSlavePositionRota = ulatorSlave.getEncoder().getPosition();
    inputs.ulatorSlaveVelocityRPM = ulatorSlave.getEncoder().getVelocity();
    // TODO applided voltage can't be pulled strait from motor. must do later
    inputs.ulatorSlaveAppliedVoltage = 0;
    inputs.ulatorSlaveOutputApms = ulatorSlave.getOutputCurrent();
    inputs.ulatorSlaveTempCelsius = ulatorSlave.getMotorTemperature();

    Logger.recordOutput("minip/ulator/speed", ulator.get());
    Logger.recordOutput("minip/ulatorSlave/speed", ulatorSlave.get());

    Logger.recordOutput("minip/sim/ulator/velocity", ulatorSim.getVelocity());
    Logger.recordOutput("minip/sim/ulatorSlave/velocity", ulatorSlaveSim.getVelocity());
  }
  /*############################## NOTE TO SELF? ##########################
   * TODO
   * look into using the motor voltage or motor draw to see if the corral has left/entered the minip
   *
   * ############################## NOTE TO SELF? ##########################*/
  @Override
  public void minipIntake() {
    ulator.set(0.3);
    ulatorSlave.set(0.3);
  }

  @Override
  public void minipSpit() {
    ulator.set(-0.3);
    ulatorSlave.set(-0.3);
  }

  @Override
  public void stopMotors() {
    ulator.stopMotor();
    ulatorSlave.stopMotor();
  }
}
