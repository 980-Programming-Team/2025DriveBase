package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class SourceManager {
  private CommandXboxController driver;

  // Pathfinding command
  private Command pathFindLeftSource;
  private Command pathFindRightSource;

  public SourceManager(int port) {
    driver = new CommandXboxController(port);
  }

  public CommandXboxController getDriver() {
    return driver;
  }

  public void configScoringPosButtons() {
    driver
        .leftTrigger()
        .whileTrue(
            pathFindLeftSource =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.CoralStation.leftCenterFace),
                    RobotContainer.constraints,
                    0));
    driver
        .rightTrigger()
        .whileTrue(
            pathFindRightSource =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.CoralStation.rightCenterFace),
                    RobotContainer.constraints,
                    0));
  }
}
