package frc.robot.subsystems.Cameras;

import java.io.File;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.Camera;

public class CameraPhoton extends CameraBase {

  // PhotonVision
  private PhotonPoseEstimator photonEstimator;
  private PhotonCamera camera;

  private Pose3d previousPose;

  public CameraPhoton(Camera cam) {

    super(cam.index);

    AprilTagFieldLayout noTrenchTagLayout;

    try {
      noTrenchTagLayout = new AprilTagFieldLayout(
          new File(Filesystem.getDeployDirectory().getPath() + "/no_trench_tags.json").getPath());
    } catch (Exception e) {
      System.out.println("TAG REVERTED TO NORMAL");
      noTrenchTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    photonEstimator = new PhotonPoseEstimator(noTrenchTagLayout, cam.robotToCam);
    camera = new PhotonCamera("PhotonCam" + String.valueOf(cam.index));
  }

  /**
   * Get the pose from PhotonVision
   */
  @Override
  public void createPose() {

    // This method is exactly why Java should have static variables in functions.

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      // get the most recent from results
      PhotonPipelineResult result = results.get(results.size() - 1);

      tagCount = result.targets.size();

      long banned = result.getTargets().stream()
          .filter(x -> VisionConstants.bannedTags.contains(x.fiducialId)).count();
      tagCount -= banned;

      confident = tagCount >= 2;

      if (result.hasTargets()) {
        // grab an estimate using the tags (returns null if no estimate)
        // remember to set photonvision to 3d mode
        Optional<EstimatedRobotPose> estimate = photonEstimator.estimateCoprocMultiTagPose(result);

        tagIDs = result.getTargets().stream().mapToLong(t -> t.fiducialId).toArray();

        // if cannot solve using multiple tags, use single tag
        // make sure that the multi tag setting is turned on
        if (estimate.isEmpty()) {
          estimate = photonEstimator.estimateLowestAmbiguityPose(result);
        }

        // set robotPose to null if there is no new estimate
        if (estimate.isEmpty()) {
          robotPose = null;
        }
        // otherwise, there is some result; set robotPose to that result
        else {

          timestamp = estimate.get().timestampSeconds;
          robotPose = estimate.get().estimatedPose;

          // Should only happen once, at the beginning.
          if (previousPose == null && robotPose != null) {
            previousPose = robotPose;
          }

          double distance = robotPose.getTranslation().getDistance(previousPose.getTranslation());

          if (distance > VisionConstants.jumpTolerance && !confident) {
            robotPose = null;
          } else {
            previousPose = robotPose;
          }
        }

        // Find most centered tag
        double bestYaw = Double.MAX_VALUE;
        for (PhotonTrackedTarget tag : result.getTargets()) {
          if (tag.getYaw() < bestYaw) {
            bestTagID = tag.fiducialId;
            cameraToTagTransform = tag.getBestCameraToTarget();
          }
        }

      }
      // if there are no targets return null
      else {
        robotPose = null;
      }
    }
    // if there are no results return null
    else {
      tagCount = 0;
      tagIDs = new long[] {};
      robotPose = null;
    }
  }
}
