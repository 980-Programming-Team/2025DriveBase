package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;

public class DriveToTagRight extends Command {
  private final Vision vision;
  private final Drive drive;
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  private final PIDController xSpeedPID = new PIDController(0.2, 0, 0.05);
  private final PIDController ySpeedPID = new PIDController(0.2, 0, 0.05);
  private final PIDController rotPID = new PIDController(0.045, 0, 0.1);

  public DriveToTagRight(Drive drive, Vision vision) {
    this.vision = vision;
    this.drive = drive;
    addRequirements(drive, vision);
  }

  @Override
  public void execute() {
    drive(false);
  }

  private float EstimateDistance() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    // put your robot at a known distance (measuring from the lens of your camera)
    // and solve the same equation for a1.
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = 60.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches =
        (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return (float) distanceFromLimelightToGoalInches;
  }

  private void drive(boolean fieldRelative) {
    double xSpeed = 0;
    double ySpeed = 0;
    double rot = 0;

    float desiredDistance = 0.2f; // 1 meter
    double range = 105.0;

    double distanceError = 0;
    double forwardSpeed = 0;

    if (LimelightHelpers.getTV("limelight")) {
      // Proportional control for distance
      distanceError = desiredDistance - EstimateDistance();
      forwardSpeed = xSpeedPID.calculate(distanceError);

      SmartDashboard.putNumber("distance", distanceError);

      rot = rotPID.calculate(LimelightHelpers.getTX("limelight"));
      rot =
          MathUtil.clamp(
              rot, Drive.getMaxAngularSpeedRadPerSec(), Drive.getMaxAngularSpeedRadPerSec());

      if (Math.abs(distanceError) > range) {
        xSpeed = xSpeedPID.calculate(0);
        ySpeed = ySpeedPID.calculate(0);
      } else if (Math.abs(distanceError) <= range) // ! close to tag
      {
        xSpeed = 0;
        ySpeed = .75;
        rot = 0;
      } else {
        xSpeed =
            MathUtil.clamp(
                forwardSpeed,
                -Drive.getMaxLinearSpeedMetersPerSec(),
                Drive.getMaxLinearSpeedMetersPerSec());
      }
      fieldRelative = false;
    } else // ! close to tag
    {
      xSpeed = 0;
      ySpeed = .75;
      rot = 0;
    }

    drive.runVelocity(xSpeed, ySpeed, rot, fieldRelative);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // drive.stop();
  }
}
