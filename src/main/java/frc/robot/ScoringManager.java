package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class ScoringManager {
  private GenericHID controller;

  // Pathfinding command
  private Command pathFindToFC;
  private Command pathFindToFR;
  private Command pathFindToBR;
  private Command pathFindToBC;
  private Command pathFindToBL;
  private Command pathFindToFL;

  public ScoringManager(int port) {
    controller = new GenericHID(port);
  }

  public GenericHID getController() {
    return controller;
  }

  public void configScoringPosButtons() {
    new JoystickButton(controller, 1)
        .whileTrue(
            pathFindToFC =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[3]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(controller, 2)
        .whileTrue(
            pathFindToFR =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[4]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(controller, 3)
        .whileTrue(
            pathFindToBR =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[5]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(controller, 4)
        .whileTrue(
            pathFindToBC =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[0]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(controller, 5)
        .whileTrue(
            pathFindToBL =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[1]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(controller, 6)
        .whileTrue(
            pathFindToFL =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[2]),
                    RobotContainer.constraints,
                    0));
  }
}
