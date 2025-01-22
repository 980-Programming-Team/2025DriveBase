package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;

public class DriveToTag extends Command {
  private final Vision vision;
  private final Drive drive;
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public DriveToTag(Drive drive, Vision vision) {
    this.vision = vision;
    this.drive = drive;
    addRequirements(drive, vision);
  }

  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    /*var xSpeed =
        -xSpeedLimiter.calculate(MathUtil.applyDeadband(0, 0.02))
            * Drive.getMaxLinearSpeedMetersPerSec();

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -ySpeedLimiter.calculate(MathUtil.applyDeadband(0, 0.02))
            * Drive.getMaxLinearSpeedMetersPerSec();

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(0, 0.02))
            * Drive.getMaxAngularSpeedRadPerSec();

    // while the A-button is pressed, overwrite some of the driving values with the output of our
    // limelight methods
    if (LimelightHelpers.getTV("limelight")) {
      final var rot_limelight = limelightAimProportional();
      rot = rot_limelight;

      final var forward_limelight = limelightRangeProportional();
      xSpeed = forward_limelight;

      // while using Limelight, turn off field-relative driving.
      drive.runVelocity(xSpeed, ySpeed, rot, false);
    } else {
      drive.runVelocity(xSpeed, ySpeed, rot, true);
    }*/

    drive(false);
  }

  // Proportional control for aiming with Limelight
  private double limelightAimProportional() {
    double kP = 0.035;
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    targetingAngularVelocity *= Drive.getMaxAngularSpeedRadPerSec();
    targetingAngularVelocity *= -1.0;
    return targetingAngularVelocity;
  }

  // Proportional control for ranging with Limelight
  private double limelightRangeProportional() {
    double kP = 0.1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= Drive.getMaxLinearSpeedMetersPerSec();
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
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
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches =
        (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return (float) distanceFromLimelightToGoalInches;
  }

  private void drive(boolean fieldRelative) {

    var xSpeed =
        -xSpeedLimiter.calculate(MathUtil.applyDeadband(0, 0.02))
            * Drive.getMaxLinearSpeedMetersPerSec();

    var ySpeed =
        -ySpeedLimiter.calculate(MathUtil.applyDeadband(0, 0.02))
            * Drive.getMaxLinearSpeedMetersPerSec();

    var rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(0, 0.02))
            * Drive.getMaxAngularSpeedRadPerSec();
    float desiredDistance = 0.2f; // 1 meter
    float range = 0.05f;
    float currentDistance = EstimateDistance(); // see the 'Case Study: Estimating Distance'
    if (LimelightHelpers.getTV("limelight")) {
      // Proportional control for distance
      double distanceError = desiredDistance - currentDistance;
      float kPDistance = 0.5f; // Proportional gain for distance control
      double forwardSpeed = kPDistance * distanceError;

      // Limit the forward speed to the maximum speed
      forwardSpeed =
          MathUtil.clamp(
              forwardSpeed,
              -Drive.getMaxLinearSpeedMetersPerSec(),
              Drive.getMaxLinearSpeedMetersPerSec());

      // Proportional control for aiming
      final var rot_limelight = limelightAimProportional();
      rot = rot_limelight;

      xSpeed = forwardSpeed;

      // If the robot is close enough to the target, stop
      if (Math.abs(distanceError) < range) { // Threshold for stopping
        xSpeed = 0;
        ySpeed = 0;
        rot = 0;
      }

      fieldRelative = false;
    }

    drive.runVelocity(xSpeed, ySpeed, rot, fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
