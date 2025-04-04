package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.AllianceFlipUtil;

public class ScoringManager {
  private GenericHID leftController;
  private GenericHID rightController;

  // Pathfinding commands
  private Command pathFindToBCL;
  private Command pathFindToBCR;
  private Command pathFindToBLL;
  private Command pathFindToBLR;
  private Command pathFindToFLL;
  private Command pathFindToFLR;
  private Command pathFindToFCL;
  private Command pathFindToFCR;
  private Command pathFindToFRL;
  private Command pathFindToFRR;
  private Command pathFindToBRL;
  private Command pathFindToBRR;

  private Superstructure superStructure;

  public ScoringManager(int leftPort, int rightPort, Superstructure superstructure) {
    leftController = new Joystick(leftPort);
    rightController = new Joystick(rightPort);
    superStructure = superstructure;
  }

  public GenericHID getLeftController() {
    return leftController;
  }

  public GenericHID getRightController() {
    return rightController;
  }

  public void configScoringPosButtons() {

    if (DriverStation.getAlliance().isPresent() /*&& DriverStation.getAlliance().get() != null*/) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        new JoystickButton(leftController, 8)
            .whileTrue(
                pathFindToBCL =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[0]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 7)
            .whileTrue(
                pathFindToBCR =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[1]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 9)
            .whileTrue(
                pathFindToBLL =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[2]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 10)
            .whileTrue(
                pathFindToBLR =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[3]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 12)
            .whileTrue(
                pathFindToFLL =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[5]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 11)
            .whileTrue(
                pathFindToFLR =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[4]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 5)
            .whileTrue(
                pathFindToFCL =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[6]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 6)
            .whileTrue(
                pathFindToFCR =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[7]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 3)
            .whileTrue(
                pathFindToFRL =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[8]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 4)
            .whileTrue(
                pathFindToFRR =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[9]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 1)
            .whileTrue(
                pathFindToBRL =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[10]),
                        RobotContainer.constraints,
                        0));
        new JoystickButton(leftController, 2)
            .whileTrue(
                pathFindToBRR =
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[11]),
                        RobotContainer.constraints,
                        0));
      } else {
        new JoystickButton(leftController, 8)
            .whileTrue(
                pathFindToBCL =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[0]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 7)
            .whileTrue(
                pathFindToBCR =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[1]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 9)
            .whileTrue(
                pathFindToBLL =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[2]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 10)
            .whileTrue(
                pathFindToBLR =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[3]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 12)
            .whileTrue(
                pathFindToFLL =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[5]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 11)
            .whileTrue(
                pathFindToFLR =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[4]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 5)
            .whileTrue(
                pathFindToFCL =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[6]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 6)
            .whileTrue(
                pathFindToFCR =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[7]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 3)
            .whileTrue(
                pathFindToFRL =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[8]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 4)
            .whileTrue(
                pathFindToFRR =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[9]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 1)
            .whileTrue(
                pathFindToBRL =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[10]), RobotContainer.constraints, 0));
        new JoystickButton(leftController, 2)
            .whileTrue(
                pathFindToBRR =
                    AutoBuilder.pathfindToPose(
                        (FieldConstants.Reef.centerFaces[11]), RobotContainer.constraints, 0));
      }
    }

    new JoystickButton(rightController, 7)
        .onTrue(
            new InstantCommand(
                    () -> {
                      superStructure.requestLevel(1);
                      superStructure.requestPreScore();
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 9)
        .onTrue(
            new InstantCommand(
                    () -> {
                      superStructure.requestPreFeed();
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 4)
        .onTrue(
            new InstantCommand(
                    () -> {
                      superStructure.requestLevel(2);
                      superStructure.requestPreScore();
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 3)
        .onTrue(
            new InstantCommand(
                    () -> {
                      superStructure.requestLevel(3);
                      superStructure.requestPreScore();
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 2)
        .onTrue(
            new InstantCommand(
                    () -> {
                      superStructure.requestLevel(4);
                      superStructure.requestPreScore();
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 8)
        .onTrue(
            new InstantCommand(
                    () -> {
                      superStructure.requestScore();
                    })
                .ignoringDisable(true));
    new JoystickButton(rightController, 12)
        .onTrue(
            new InstantCommand(
                    () -> {
                      superStructure.candle.clearAllAnims();
                      //   superStructure.funnel.requestPosition(5300);
                    })
                .ignoringDisable(true));
  }
}
