package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.cam1Name;
import static frc.robot.subsystems.vision.VisionConstants.cam1RobotToCam;
import static frc.robot.subsystems.vision.VisionConstants.kTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {

  private final PhotonCamera camera1;
  private final PhotonPoseEstimator camera1Estimator;

  private Pose2d lastEstimate = new Pose2d();

  public VisionIOPhoton() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Camera 1
    camera1 = new PhotonCamera(cam1Name);
    camera1Estimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam1RobotToCam);
    camera1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    SmartDashboard.putBoolean("KillSideCams", false);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
    lastEstimate = currentEstimate;

    PhotonPipelineResult[] results = getAprilTagResults();
    PhotonPoseEstimator[] photonEstimators = getAprilTagEstimators(currentEstimate);

    inputs.estimate = new Pose2d[] {new Pose2d()};

    // add code to check if the closest target is in front or back
    inputs.timestamp = estimateLatestTimestamp(results);

    if (hasEstimate(results)) {
      // inputs.results = results;
      inputs.estimate = getEstimatesArray(results, photonEstimators);
      inputs.hasEstimate = true;

      int[][] cameraTargets = getCameraTargets(results);
      inputs.camera1Targets = cameraTargets[0];

      Pose3d[] tags = getTargetsPositions(results);
      Logger.recordOutput("Vision/Targets3D", tags);
      Logger.recordOutput("Vision/Targets", Pose3dToPose2d(tags));
      Logger.recordOutput("Vision/TagCounts", tagCounts(results));
    } else {
      inputs.timestamp = inputs.timestamp;
      inputs.hasEstimate = false;
    }

    // Log if the robot code can see these cameras
    Logger.recordOutput("Vision/cam1/Connected", camera1.isConnected());
  }

  private PhotonPipelineResult[] getAprilTagResults() {
    PhotonPipelineResult cam1_result = getLatestResult(camera1);

    printStuff("cam1", cam1_result);

    return new PhotonPipelineResult[] {cam1_result};
  }

  private void printStuff(String name, PhotonPipelineResult result) {
    Logger.recordOutput("Vision/" + name + "/results", result.getTargets().size());

    PhotonTrackedTarget target = result.getBestTarget();
    if (target != null) {
      Logger.recordOutput(
          "Vision/" + name + "/PoseAmbiguity", result.getBestTarget().getPoseAmbiguity());
      Logger.recordOutput("Vision/" + name + "/Yaw", result.getBestTarget().getYaw());
    }
  }

  private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {

    camera1Estimator.setReferencePose(currentEstimate);

    return new PhotonPoseEstimator[] {camera1Estimator};
  }

  @Override
  public boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
    /*
     * && kTagLayout.
     * getTagPose(
     * result.
     * getBestTarget().
     * getFiducialId())
     * .get().toPose2d(
     * ).getTranslation
     * ()
     * .getDistance(
     * lastEstimate.
     * getTranslation()
     * ) < MAX_DISTANCE
     */ ;
  }
}
