package frc.robot.subsystems;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.VisionConstants;

public class VisionCoprocessor extends Vision {

  private DoubleArraySubscriber cameraGlobalPose; // "Global Pose" [x, y, yaw]
  // private DoubleArraySubscriber tagToCameraPose; // "Tag To Camera Pose" [x, y, yaw]
  private IntegerSubscriber visibleTags; // "AprilTag Count"
  private DoubleSubscriber robotPoseTimestamp;// "Global Pose Timestamp"

  public VisionCoprocessor() {
    cameraGlobalPose =
        visionTable.getDoubleArrayTopic("Global Pose").subscribe(new double[] {0, 0, 0});
    // tagToCameraPose = visionTable.getDoubleArrayTopic("Tag To Camera Pose").subscribe(new
    // double[] {0, 0, 0});

    visibleTags = visionTable.getIntegerTopic("AprilTag Count").subscribe(0);

    robotPoseTimestamp = visionTable.getDoubleTopic("Global Pose Timestamp").subscribe(0.0);

    // Calibrate Unix to FPGA time offset (NS)
    // Note: the Java epoch is the same as the Unix epoch
    Instant nowInstant = Instant.now();
    long nowFpgaNs = RobotController.getFPGATime() * 1_000L;

    long nowUnixNs = nowInstant.getEpochSecond() * 1_000_000_000L + nowInstant.getNano();

    visionTable.getEntry("FPGA Offset").setValue(nowUnixNs - nowFpgaNs);
  }

  @Override
  protected void createPose() {
    tagCount = visibleTags.get();

    confident = tagCount > 1;
    // isConfident = true;

    robotPose = null;

    if (confident) {

      double[] pose = cameraGlobalPose.get(); // [x, y, theta]

      timestamp = robotPoseTimestamp.get();
      Pose3d cameraPose = new Pose3d(new Translation3d(pose[0], pose[1], pose[2]),
          new Rotation3d(pose[3], pose[4], pose[5]));

      robotPose = cameraPose.transformBy(VisionConstants.camToRobot);

    }
  }
}
