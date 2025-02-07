// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class ElevatorSim {

  private final LoggedMechanism2d panel;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d elevator;
  public final Elevator elev;

  private static final Color8Bit background = new Color8Bit(Color.kBlue);
  private static final Color8Bit elevatorColor = new Color8Bit(Color.kRed);

  public ElevatorSim(Elevator ELEV) {
    this.elev = ELEV;
    this.panel =
        new LoggedMechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(40), background);
    this.root = panel.getRoot("elevator root", Units.inchesToMeters(10), Units.inchesToMeters(0));
    this.elevator =
        root.append(
            new LoggedMechanismLigament2d(
                "elevator1",
                Units.inchesToMeters(10),
                Units.degreesToRadians(90),
                Units.inchesToMeters(10),
                elevatorColor));
  }

  public void periodic() {
    ElevatorState.inputState currState = elev.getState();
    if (currState != null) {
      this.elevator.setLength(currState.currentHightMeters());
    }

    Logger.recordOutput("robot sim", panel);
    // SmartDashboard.putData("elevator test", (Sendable) this.panel.get);
  }
}
