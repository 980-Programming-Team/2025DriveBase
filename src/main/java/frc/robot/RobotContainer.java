package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.LED.CANdleSystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
// import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
// import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOSpark;
// import frc.robot.subsystems.manipulator.Manipulator;
// import frc.robot.subsystems.manipulator.ManipulatorIOSpark;
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
  private final Vision vision;

  // Controllers
  public static SourceManager driver = new SourceManager(0);

  public static ScoringManager operatorBoard = new ScoringManager(1, 2);

  private final CommandXboxController testController = new CommandXboxController(3);

  // CANdle
  private final CANdleSystem m_candleSubsystem = new CANdleSystem();

  public static ElevatorIO elevatorIO =
      Constants.elevatorEnabled ? new ElevatorIOSpark() : new ElevatorIO() {};
  // public static ManipulatorIO armClawIO =
  // Constants.armEnabled ? new ManipulatorIOSpark() : new ManipulatorIO() {};
  public static FunnelIO funnelIO =
      Constants.funnelEnabled ? new FunnelIOSpark() : new FunnelIO() {};
  public static ClimberIO climberIO =
      Constants.climberEnabled ? new ClimberIOSpark() : new ClimberIO() {};

  public static Elevator elevator = new Elevator(elevatorIO);
  // public static Manipulator armClaw = new Manipulator(armClawIO);
  // public static Funnel funnel = new Funnel(funnelIO);
  public static Climber climber = new Climber(climberIO);
  // public static Superstructure superstructure = new Superstructure(elevator, armClaw, funnel);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Pathfinding command
  private final Command pathFindToProcessor;
  private final Command pathFindToFC;
  private final Command pathFindToBC;
  private final Command pathFindToFL;
  private final Command pathFindToBL;
  private final Command pathFindToSourceL;

  // Create the constraints to use while pathfinding
  public static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

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
                new VisionIOLimelight("limelight-front", drive::getRotation),
                new VisionIOLimelight("limelight-source", drive::getRotation));
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

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathFindToProcessor =
        AutoBuilder.pathfindToPose(
            FieldConstants.Processor.centerFace, constraints, 0.0 // Goal end velocity in meters/sec
            );

    pathFindToFC = AutoBuilder.pathfindToPose(FieldConstants.Reef.centerFaces[3], constraints, 0);
    pathFindToBC = AutoBuilder.pathfindToPose(FieldConstants.Reef.centerFaces[0], constraints, 0);
    pathFindToFL = AutoBuilder.pathfindToPose(FieldConstants.Reef.centerFaces[2], constraints, 0);
    pathFindToBL = AutoBuilder.pathfindToPose(FieldConstants.Reef.centerFaces[1], constraints, 0);
    pathFindToSourceL =
        AutoBuilder.pathfindToPose(FieldConstants.CoralStation.leftCenterFace, constraints, 0);

    // Set up auto routines
    registerNamedCommands();
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
            () -> -driver.getDriver().getLeftY(),
            () -> -driver.getDriver().getLeftX(),
            () -> -driver.getDriver().getRightX()));

    // Reset gyro to 0° when A button is pressed
    driver
        .getDriver()
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Bind the pathfinding command to a button (e.g., B button)
    driver.getDriver().b().whileTrue(pathFindToProcessor);
    driver.getDriver().x().whileTrue(pathFindToFC);
    driver.configScoringPosButtons();

    // driver
    //     .getDriver().b()
    //     .whileTrue(new PathfindingCommand(
    //         drive,
    //         () -> new Pose2d(FieldConstants.Processor.centerFace.getTranslation(), new
    // Rotation2d()),
    //         () -> drive.getPose(),
    //         LocalADStarAK.getInstance().getPathPlanner(),
    //         drive::setModuleStates,
    //         drive));

    // l1.onTrue(new RunCommand(m_candleSubsystem::SetLEDRed, m_candleSubsystem));
    // l2.onTrue(new RunCommand(m_candleSubsystem::SetLEDGreen, m_candleSubsystem));
    // l3.onTrue(new RunCommand(m_candleSubsystem::SetLEDYellow, m_candleSubsystem));
    // l4.onTrue(new RunCommand(m_candleSubsystem::SetLEDBlue, m_candleSubsystem));

  }

  public void registerNamedCommands() {

    NamedCommands.registerCommand("BC", (pathFindToBC));
    NamedCommands.registerCommand("FC", (pathFindToFC));
    NamedCommands.registerCommand("FL", (pathFindToFL));
    NamedCommands.registerCommand("BL", (pathFindToBL));
    NamedCommands.registerCommand("SourceL", (pathFindToSourceL));
    NamedCommands.registerCommand("Processor", (pathFindToProcessor));

    /*
     // FRONT CENTER ALIGN LEFT L2
     NamedCommands.registerCommand("FC AL L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT CENTER ALIGN LEFT L3
     NamedCommands.registerCommand("FC AL L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT CENTER ALIGN LEFT L4
     NamedCommands.registerCommand("FC AL L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT CENTER ALIGN RIGHT L2
     NamedCommands.registerCommand("FC AR L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT CENTER ALIGN RIGHT L3
     NamedCommands.registerCommand("FC AR L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT CENTER ALIGN RIGHT L4
     NamedCommands.registerCommand("FC AR L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT RIGHT ALIGN LEFT L2
     NamedCommands.registerCommand("FR AL L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT RIGHT ALIGN LEFT L3
     NamedCommands.registerCommand("FR AL L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT RIGHT ALIGN LEFT L4
     NamedCommands.registerCommand("FR AL L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT RIGHT ALIGN RIGHT L2
     NamedCommands.registerCommand("FR AR L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT RIGHT ALIGN RIGHT L3
     NamedCommands.registerCommand("FR AR L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT RIGHT ALIGN RIGHT L4
     NamedCommands.registerCommand("FR AR L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK RIGHT ALIGN LEFT L2
     NamedCommands.registerCommand("BR AL L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK RIGHT ALIGN LEFT L3
     NamedCommands.registerCommand("BR AL L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK RIGHT ALIGN LEFT L4
     NamedCommands.registerCommand("BR AL L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK RIGHT ALIGN RIGHT L2
     NamedCommands.registerCommand("BR AR L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK RIGHT ALIGN RIGHT L3
     NamedCommands.registerCommand("BR AR L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK RIGHT ALIGN RIGHT L4
     NamedCommands.registerCommand("BR AR L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK CENTER ALIGN LEFT L2
     NamedCommands.registerCommand("BC AL L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK CENTER ALIGN LEFT L3
     NamedCommands.registerCommand("BC AL L3", new InstantCommand(
         () -> {
           superstructure.requestEject();
         }));

     // BACK CENTER ALIGN LEFT L4
     NamedCommands.registerCommand("BC AL L4", new InstantCommand(
         () -> {
           superstructure.requestEject();
         }));

     // BACK CENTER ALIGN RIGHT L2
     NamedCommands.registerCommand("BC AR L2", new InstantCommand(
         () -> {
           superstructure.requestEject();
         }));

     // BACK CENTER ALIGN RIGHT L3
     NamedCommands.registerCommand("BC AR L3", new InstantCommand(
         () -> {
           superstructure.requestEject();
         }));

     // BACK CENTER ALIGN RIGHT L4
     NamedCommands.registerCommand("BC AR L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK LEFT ALIGN LEFT L2
     NamedCommands.registerCommand("BL AL L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK LEFT ALIGN LEFT L3
     NamedCommands.registerCommand("BL AL L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK LEFT ALIGN LEFT L4
     NamedCommands.registerCommand("BL AL L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK LEFT ALIGN RIGHT L2
     NamedCommands.registerCommand("BL AR L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK LEFT ALIGN RIGHT L3
     NamedCommands.registerCommand("BL AR L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // BACK LEFT ALIGN RIGHT L4
     NamedCommands.registerCommand("BL AR L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

    // FRONT LEFT ALIGN LEFT L2
    NamedCommands.registerCommand("FL AL L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT LEFT ALIGN LEFT L3
     NamedCommands.registerCommand("FL AL L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT LEFT ALIGN LEFT L4
     NamedCommands.registerCommand("FL AL L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT LEFT ALIGN RIGHT L2
     NamedCommands.registerCommand("FL AR L2", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT LEFT ALIGN RIGHT L3
     NamedCommands.registerCommand("FL AR L3", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));

     // FRONT LEFT ALIGN RIGHT L4
     NamedCommands.registerCommand("FL AR L4", new InstantCommand(
       () -> {
         superstructure.requestEject();
       }));
       */
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
