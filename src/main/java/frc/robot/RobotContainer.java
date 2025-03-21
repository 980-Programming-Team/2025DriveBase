package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.LED.CANdleSystem;
import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.climber.Climber;
// import frc.robot.subsystems.climber.ClimberIO;
// import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.funnel.Funnel;
// import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOSpark;
import frc.robot.subsystems.manipulator.Arm;
import frc.robot.subsystems.manipulator.ArmIO;
import frc.robot.subsystems.manipulator.ArmIOSpark;
import frc.robot.subsystems.manipulator.Claw;
import frc.robot.subsystems.manipulator.ClawIO;
import frc.robot.subsystems.manipulator.ClawIOSpark;
// import frc.robot.subsystems.manipulator.Manipulator;
// import frc.robot.subsystems.manipulator.ManipulatorIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;

import java.util.concurrent.CancellationException;

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

  public static ElevatorIO elevatorIO =
      Constants.elevatorEnabled ? new ElevatorIOSpark() : new ElevatorIO() {};
  public static ArmIO armIO = Constants.armEnabled ? new ArmIOSpark() : new ArmIO() {};
  public static ClawIO clawIO = Constants.armEnabled ? new ClawIOSpark() : new ClawIO() {};
  public static ClimberIO climberIO =
      Constants.climberEnabled ? new ClimberIOSpark() : new ClimberIO() {};
  public static FunnelIO funnelIO =
      Constants.funnelEnabled ? new FunnelIOSpark() : new FunnelIO() {};
  // public static ClimberIO climberIO =
  // Constants.climberEnabled ? new ClimberIOSpark() : new ClimberIO() {};

  public static Elevator elevator = new Elevator(elevatorIO);
  public static Arm arm = new Arm(armIO);
  public static Claw claw = new Claw(clawIO);
  public static Funnel funnel = new Funnel(funnelIO);
  public static Climber climber = new Climber(climberIO);
  // public static Climber climber = new Climber(climberIO);
  private static CANdleSystem candle = new CANdleSystem();
  public static Superstructure superstructure =
      new Superstructure(elevator, arm, claw, funnel, /*climber, */candle);

  // Controllers
  public static SourceManager driver = new SourceManager(0, superstructure);

  public static ScoringManager operatorBoard = new ScoringManager(1, 2, superstructure);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // Create the constraints to use while pathfinding
  public static PathConstraints constraints =
      new PathConstraints(2.0, 1.25, Units.degreesToRadians(540), Units.degreesToRadians(720));
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
                new VisionIOLimelight(VisionConstants.limelightFront, drive::getRotation),
                new VisionIOLimelight(VisionConstants.limelightSource, drive::getRotation));
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
                    VisionConstants.limelightFront, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.limelightSource,
                    VisionConstants.robotToCamera1,
                    drive::getPose));
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

    driver.configScoringPosButtons();
    operatorBoard.configScoringPosButtons();
  }

  public void registerNamedCommands() {

    NamedCommands.registerCommand(
        "L4",
        (new InstantCommand(
            () -> {
              superstructure.requestLevel(4);
              superstructure.requestPreScore();
            })));

    NamedCommands.registerCommand(
        "FLR",
        (AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[5]),
            RobotContainer.constraints,
            0)));

    NamedCommands.registerCommand(
        "BCL",
        (AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[0]),
            RobotContainer.constraints,
            0)));

    NamedCommands.registerCommand(
        "FRL",
        (AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[8]),
            RobotContainer.constraints,
            0)));

    NamedCommands.registerCommand(
        "FCR",
        (AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[7]),
            RobotContainer.constraints,
            0)));

    NamedCommands.registerCommand(
        "BRR",
        (AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[11]),
            RobotContainer.constraints,
            0)));

    NamedCommands.registerCommand(
        "Shoot",
        (new InstantCommand(
            () -> {
              superstructure.requestScore();
            })));

    NamedCommands.registerCommand(
        "L3",
        (new InstantCommand(
            () -> {
              superstructure.requestLevel(3);
              superstructure.requestPreScore();
            })));

    NamedCommands.registerCommand(
        "L1",
        (new InstantCommand(
            () -> {
              superstructure.requestLevel(1);
              superstructure.requestPreScore();
            })));

    NamedCommands.registerCommand(
        "Feed",
        (new InstantCommand(
            () -> {
              superstructure.intakeCoral(true, 5);
              superstructure.requestPreScore();
            })));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void autoInit() {
    isInMatch = true;
  }

  public void disabledInit() {}

  DriverStation.Alliance allianceColor = Alliance.Red;
  boolean isInMatch;

  public void disabledPeriodic() {
    if (!isInMatch) {
      DriverStation.getAlliance()
          .ifPresent(
              a -> {
                if (a != allianceColor) {
                  allianceColor = a;
                  operatorBoard.configScoringPosButtons();
                  System.out.println("Buttons configured for " + a.name() + " Alliance");
                }
              });
    }
    // System.out.println("Buttons configured for " + allianceColor.name() + " Alliance");
  }
}
