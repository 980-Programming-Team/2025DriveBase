package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    var xSpeed =
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
    }
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

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
