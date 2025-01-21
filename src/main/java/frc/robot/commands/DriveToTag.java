package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class DriveToTag extends Command {
  private final Drive drive;
  private final Vision vision;
  private final int tagID;
  private final double distanceMeters;
  private Pose2d targetPose;

  public DriveToTag(Drive drive, Vision vision, int tagID, double distanceMeters) {
    this.drive = drive;
    this.vision = vision;
    this.tagID = tagID;
    this.distanceMeters = distanceMeters;
    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    Pose2d tagPose = vision.getTagPose(tagID);
    if (tagPose != null) {
      Translation2d targetTranslation =
          tagPose
              .getTranslation()
              .minus(new Translation2d(distanceMeters, 0).rotateBy(tagPose.getRotation()));
      targetPose = new Pose2d(targetTranslation, tagPose.getRotation());
      System.out.println("Target Pose: " + targetPose);
    } else {
      targetPose = null;
      System.out.println("Tag not found");
    }
  }

  @Override
  public void execute() {
    if (targetPose != null) {
      System.out.println("Driving to Pose: " + targetPose);
      drive.driveToPose(targetPose);
    }
  }

  @Override
  public boolean isFinished() {
    if (targetPose == null) {
      return true;
    }
    Pose2d currentPose = drive.getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    return distance < 0.1; // Consider the command finished if within 10 cm of the target
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
