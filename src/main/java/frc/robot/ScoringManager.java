package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class ScoringManager {
  private GenericHID leftController;
  private GenericHID rightController;

  // Pathfinding command
  private Command pathFindToFC;
  private Command pathFindToFR;
  private Command pathFindToBR;
  private Command pathFindToBC;
  private Command pathFindToBL;
  private Command pathFindToFL;

  public ScoringManager(int leftPort, int rightPort) {
    leftController = new GenericHID(leftPort);
    rightController = new GenericHID(rightPort);
  }

  public GenericHID getLeftController() {
    return leftController;
  }

  public GenericHID getRightController() {
    return rightController;
  }

  public void configScoringPosButtons() {
    new JoystickButton(leftController, 1)
        .whileTrue(
            pathFindToFC =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[3]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(leftController, 2)
        .whileTrue(
            pathFindToFR =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[4]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(leftController, 3)
        .whileTrue(
            pathFindToBR =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[5]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(leftController, 4)
        .whileTrue(
            pathFindToBC =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[0]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(leftController, 5)
        .whileTrue(
            pathFindToBL =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[1]),
                    RobotContainer.constraints,
                    0));
    new JoystickButton(leftController, 6)
        .whileTrue(
            pathFindToFL =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[2]),
                    RobotContainer.constraints,
                    0));
  }
}
