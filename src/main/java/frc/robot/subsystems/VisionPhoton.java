package frc.robot.subsystems;

import java.io.File;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.VisionConstants;

public class VisionPhoton extends Vision {

  // PhotonVision
  private PhotonPoseEstimator photonEstimator;
  private PhotonCamera camera;

  private Pose3d previousPose;

  public VisionPhoton() {

    AprilTagFieldLayout noTrenchTagLayout;

    try {
      noTrenchTagLayout = new AprilTagFieldLayout(
          new File(Filesystem.getDeployDirectory().getPath() + "/no_trench_tags.json").getPath());
    } catch (Exception e) {
      System.out.println("TAG REVERTED TO NORMAL");
      noTrenchTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    photonEstimator = new PhotonPoseEstimator(noTrenchTagLayout, VisionConstants.robotToCam);
    camera = new PhotonCamera("PhotonCamera");
  }

  /**
   * Get the pose from PhotonVision
   */
  @Override
  protected void createPose() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      // get the most recent from results
      PhotonPipelineResult result = results.get(results.size() - 1);

      tagCount = result.targets.size();

      long banned = result.getTargets().stream()
          .filter(x -> VisionConstants.bannedTags.contains(x.fiducialId)).count();
      tagCount -= banned;

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

          // Should only happen once
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

      }
      // if there are no targets return null
      else {
        robotPose = null;
      }
    }
    // if there are no results return null
    else {
      tagCount = 0;
      robotPose = null;
    }
  }
}
