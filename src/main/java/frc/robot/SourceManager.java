package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class SourceManager {
  private CommandXboxController driver;

  // Pathfinding command
  private Command pathFindLeftSource;
  private Command pathFindRightSource;

  private Superstructure superStructure;

  public SourceManager(int port, Superstructure superstructure) {
    driver = new CommandXboxController(port);

    superStructure = superstructure;
  }

  public CommandXboxController getDriver() {
    return driver;
  }

  public void configScoringPosButtons() {
    /*driver
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
                    0));*/

    // default command allows multiple objects to be reached and checked periodically
    // -> can only have one per class
    superStructure.setDefaultCommand(
        Commands.run(() -> superStructure.intakeCoral(driver.leftBumper()), superStructure));
  }
}
