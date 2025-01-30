package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToTagLeft;
import frc.robot.commands.DriveToTagRight;
import frc.robot.commands.IndexIntoShooter;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.Reef.LevelFour;
import frc.robot.commands.Reef.LevelOne;
import frc.robot.commands.Reef.LevelThree;
import frc.robot.commands.Reef.LevelTwo;
import frc.robot.commands.Reef.ResetElevator;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final Elevator elevator;
  private final Collector collector;
  private final Shooter shooter;

  private final Vision vision;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);

  // Haute M42
  private final Joystick gamePad = new Joystick(1);

  private JoystickButton up = new JoystickButton(gamePad, 8);
  private JoystickButton down = new JoystickButton(gamePad, 7);

  private JoystickButton l1 = new JoystickButton(gamePad, 2);
  private JoystickButton l2 = new JoystickButton(gamePad, 3);
  private JoystickButton l3 = new JoystickButton(gamePad, 1);
  private JoystickButton l4 = new JoystickButton(gamePad, 4);

  private JoystickButton button5 = new JoystickButton(gamePad, 5);
  private JoystickButton button6 = new JoystickButton(gamePad, 6);
  private JoystickButton button9 = new JoystickButton(gamePad, 9);
  private JoystickButton button10 = new JoystickButton(gamePad, 10);

  private JoystickButton reset = new JoystickButton(gamePad, 13);

  private JoystickButton button11 = new JoystickButton(gamePad, 11);
  private JoystickButton button12 = new JoystickButton(gamePad, 12);
  private JoystickButton button14 = new JoystickButton(gamePad, 14);

  // 15 16

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
                new VisionIOLimelight("limelight", drive::getRotation));
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
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    elevator = new Elevator(drive);
    collector = new Collector(drive);
    shooter = new Shooter(drive);

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

    elevator.setDefaultCommand(Commands.run(() -> elevator.Manual(gamePad.getZ()), elevator));

    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.a().onTrue(new LevelTwoPID());

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.y().onTrue(new ResetElevator(elevator));

    // Bind the DriveToTag command to the left bumper with a desired distance of 2 meters and target
    // tag IDs

    controller.leftBumper().whileTrue(new DriveToTagLeft(drive, vision));
    controller.rightBumper().whileTrue(new DriveToTagRight(drive, vision));

    controller
        .povLeft()
        .onTrue(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> .5, () -> 0)));

    controller
        .povRight()
        .onTrue(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> -.5, () -> 0)));

    // gamePad
    l1.onTrue(new LevelOne(elevator));
    l2.onTrue(new LevelTwo(elevator));
    l3.onTrue(new LevelThree(elevator));
    l4.onTrue(new LevelFour(elevator));

    reset.onTrue(new ResetElevator(elevator));

    button5.whileTrue(new Intake(collector));
    button6.whileTrue(new Outtake(collector));

    button9.whileTrue(new IndexIntoShooter(collector));
    button10.whileTrue(new ShooterCommand(shooter));

    // controller.rightTrigger().whileTrue(new IndexIntoShooter(collector));
    // controller.leftTrigger().whileTrue(new ShooterCommand(shooter));

    // button11.whileTrue(new IndexIntoShooterAndShoot(collector, shooter));

    button14.whileTrue(Commands.run(() -> collector.off(), collector));
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
