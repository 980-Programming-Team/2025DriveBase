package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import java.util.Set;
import java.util.function.Supplier;

public class DriveToTag extends Command {
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(8, 8);

  private final Vision vision;
  private final Drive drive;
  private final Supplier<Pose2d> poseProvider;
  private final double desiredDistance; // New field for desired distance
  private final Set<Integer> targetTagIds; // New field for target tag IDs

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private TargetObservation lastTarget;

  public DriveToTag(
      Drive drive,
      Vision vision,
      Supplier<Pose2d> poseProvider,
      double desiredDistance,
      Set<Integer> targetTagIds) {
    this.vision = vision;
    this.drive = drive;
    this.poseProvider = poseProvider;
    this.desiredDistance = desiredDistance; // Initialize desired distance
    this.targetTagIds = targetTagIds; // Initialize target tag IDs

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    vision.setTargetTagIds(targetTagIds); // Set the target tag IDs
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();

    var inputs = vision.getInputs();
    if (inputs.connected) {
      // Use the latest target observation
      lastTarget = inputs.latestTargetObservation;

      // Calculate the goal pose based on the target observation and desired distance
      double currentDistance =
          Math.hypot(lastTarget.tx().getRadians(), lastTarget.ty().getRadians());
      double distanceError = currentDistance - desiredDistance;
      double goalX = robotPose.getX() + distanceError * Math.cos(lastTarget.tx().getRadians());
      double goalY = robotPose.getY() + distanceError * Math.sin(lastTarget.ty().getRadians());

      var goalPose =
          new Pose2d(goalX, goalY, Rotation2d.fromDegrees(0)); // Assuming we want to face forward

      // Drive
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      omegaController.setGoal(goalPose.getRotation().getRadians());
    }

    if (lastTarget == null) {
      // No target has been visible
      drive.stop();
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
