package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  // private final Motor motor;

  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);
  public static final Joystick driver = new Joystick(0);

  private final JoystickButton buttonA = new JoystickButton(driver, 1);
  private final JoystickButton buttonB = new JoystickButton(driver, 2);

  @SuppressWarnings("unused")
  private final JoystickButton rightPaddle = new JoystickButton(driver, 3);

  private final JoystickButton buttonX = new JoystickButton(driver, 4);
  private final JoystickButton buttonY = new JoystickButton(driver, 5);

  @SuppressWarnings("unused")
  private final JoystickButton leftPaddle = new JoystickButton(driver, 6);

  private final JoystickButton buttonLB = new JoystickButton(driver, 7);

  @SuppressWarnings("unused")
  private final JoystickButton buttonRB = new JoystickButton(driver, 8);

  @SuppressWarnings("unused")
  private final JoystickButton buttonLT = new JoystickButton(driver, 9);

  @SuppressWarnings("unused")
  private final JoystickButton buttonRT = new JoystickButton(driver, 10);

  @SuppressWarnings("unused")
  private final JoystickButton buttonStart = new JoystickButton(driver, 11);

  @SuppressWarnings("unused")
  private final JoystickButton buttonBack = new JoystickButton(driver, 12);

  @SuppressWarnings("unused")
  private final JoystickButton homeButton = new JoystickButton(driver, 13);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    camera0Name, robotToCamera0)); // Using a default Transform3d

        // motor = new Motor("leftElevatorMotor", new MotorIOTalonFX(0, "rio", 40, false, true, 0));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    camera0Name, robotToCamera0)); // Using a default Transform3d
        // drive::addVisionMeasurement,
        // new VisionIOPhotonVisionSim(
        //  camera0Name, robotToCamera0, drive::getPose)); // Default Vision for SIM

        // motor = new Motor("leftElevatorMotor", new MotorIOSim(DCMotor.getFalcon500(1), 0.2,
        // 0.1));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    camera0Name, robotToCamera0)); // Default Vision for DEFAULT

        // motor = new Motor("leftElevatorMotor", new MotorIOSim(DCMotor.getFalcon500(1), 1, 0.1));

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driver.getRawAxis(1) / 2,
            () -> driver.getRawAxis(0) / 2,
            () -> -driver.getRawAxis(3) / 2));

    // Lock to 0° when A button is held
    buttonA.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> driver.getRawAxis(1) * Constants.invert,
            () -> driver.getRawAxis(0) * Constants.invert,
            () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    buttonB.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    buttonY.whileTrue(
        DriveCommands.alignToPoseAndAngleCom(
            drive,
            () -> new Translation2d(11.45, 7.55),
            () -> new Rotation2d(Math.toRadians(-90))));

    buttonX.whileTrue(
        DriveCommands.alignToFeederCom(
            drive,
            () -> -driver.getRawAxis(1),
            () -> -driver.getRawAxis(0),
            () -> -driver.getRawAxis(3)));

    // buttonLB.whileTrue(DriveCommands.moveMotorTestCom(motor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
