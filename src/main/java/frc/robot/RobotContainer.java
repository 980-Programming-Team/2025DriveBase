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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Reef.LevelFour;
import frc.robot.commands.Reef.LevelOne;
import frc.robot.commands.Reef.LevelThree;
import frc.robot.commands.Reef.LevelTwo;
import frc.robot.commands.Reef.ResetElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);

  // Haute M42
  private final Joystick gamePad = new Joystick(1);

  private JoystickButton up = new JoystickButton(gamePad, 8);
  private JoystickButton down = new JoystickButton(gamePad, 7);

  private JoystickButton lL1 = new JoystickButton(gamePad, 2);
  private JoystickButton lL2 = new JoystickButton(gamePad, 3);
  private JoystickButton lL3 = new JoystickButton(gamePad, 1);
  private JoystickButton lL4 = new JoystickButton(gamePad, 4);

  private JoystickButton rL1 = new JoystickButton(gamePad, 5);
  private JoystickButton rl2 = new JoystickButton(gamePad, 6);
  private JoystickButton rl3 = new JoystickButton(gamePad, 9);
  private JoystickButton rl4 = new JoystickButton(gamePad, 10);

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
        break;
    }

    elevator = new Elevator(drive);

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
            () -> controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    // gamePad
    lL1.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> .5, () -> 0)),
            new LevelOne(elevator)));
    lL2.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> .5, () -> 0)),
            new LevelTwo(elevator)));
    lL3.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> .5, () -> 0)),
            new LevelThree(elevator)));
    lL4.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> .5, () -> 0)),
            new LevelFour(elevator)));

    rL1.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> -.5, () -> 0)),
            new LevelOne(elevator)));
    rl2.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> -.5, () -> 0)),
            new LevelTwo(elevator)));
    rl3.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> -.5, () -> 0)),
            new LevelThree(elevator)));
    rl4.onTrue(
        new SequentialCommandGroup(
            Commands.deadline(
                new WaitCommand(.8),
                DriveCommands.joystickDrive(drive, () -> 0, () -> -.5, () -> 0)),
            new LevelFour(elevator)));

    reset.onTrue(new ResetElevator(elevator));

    // up.onTrue(getAutonomousCommand());
    // down.onTrue(getAutonomousCommand());

    // gamePad.povUp(null);
    // gamePad.povDown(null);
    // gamePad.povLeft(null);
    // gamePad.povUpRight(null);

    // gamePad.trigger(null);
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
