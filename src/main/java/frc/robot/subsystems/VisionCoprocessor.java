package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.Constants.VisionConstants;

public class VisionCoprocessor extends Vision {
  // "Global Pose" [x, y, z, roll, pitch, yaw]
  private DoubleArraySubscriber cameraGlobalPoseSubscriber;
  // "Tag To Camera Pose" [x, y, z, roll, pitch, yaw]
  private DoubleArraySubscriber tagToCameraTransformSubscriber;
  private IntegerArraySubscriber visibleTagsSubscriber;

  public VisionCoprocessor() {
    cameraGlobalPoseSubscriber =
        visionTable.getDoubleArrayTopic("Global Pose").subscribe(new double[] {0, 0, 0, 0, 0, 0});
    cameraGlobalPoseSubscriber = visionTable.getDoubleArrayTopic("Tag To Camera Pose")
        .subscribe(new double[] {0, 0, 0, 0, 0, 0});

    visibleTagsSubscriber = visionTable.getIntegerArrayTopic("AprilTags").subscribe(new long[] {});
  }

  /**
   * Create pose estimates from Coprocessor data
   */
  @Override
  public void createPose() {

    tagIDs = visibleTagsSubscriber.get();
    tagCount = tagIDs.length;

    confident = tagCount > 1;

    robotPose = null;
    tagToCameraTransform = null;

    if (confident) {

      // Get data from network tables (including timestamp)
      TimestampedDoubleArray data = cameraGlobalPoseSubscriber.getAtomic();

      // [0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw]
      double[] poseData = data.value;
      timestamp = data.timestamp;

      Pose3d cameraPose = new Pose3d(new Translation3d(poseData[0], poseData[1], poseData[2]),
          new Rotation3d(poseData[3], poseData[4], poseData[5]));

      robotPose = cameraPose.transformBy(VisionConstants.camToRobot);

    }

    if (tagCount > 0) {
      // [0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw]
      double[] transformData = tagToCameraTransformSubscriber.get();
      tagToCameraTransform =
          new Transform3d(new Translation3d(transformData[0], transformData[1], transformData[2]),
              new Rotation3d(transformData[3], transformData[4], transformData[5]));
    }
  }
}
