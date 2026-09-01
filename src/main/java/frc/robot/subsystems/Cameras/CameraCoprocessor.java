package frc.robot.subsystems.Cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.Constants.VisionConstants.Camera;

public class CameraCoprocessor extends CameraBase {
  // "Global Pose" [x, y, z, w, x, y, z]
  private DoubleArraySubscriber cameraGlobalPoseSubscriber;
  // "Tag To Camera Pose" [x, y, z, w, x, y, z]
  private DoubleArraySubscriber tagToCameraTransformSubscriber;
  private IntegerArraySubscriber visibleTagsSubscriber;
  private DoubleArraySubscriber deviationSubscriber;
  private Transform3d camToRobot;

  public CameraCoprocessor(Camera cam) {
    super(cam.index);

    cameraGlobalPoseSubscriber =
        visionTable.getDoubleArrayTopic("Global Pose").subscribe(new double[7]);

    tagToCameraTransformSubscriber =
        visionTable.getDoubleArrayTopic("Tag To Camera Pose").subscribe(new double[7]);

    visibleTagsSubscriber = visionTable.getIntegerArrayTopic("AprilTags").subscribe(new long[] {});

    deviationSubscriber =
        visionTable.getDoubleArrayTopic("Standard Deviations").subscribe(new double[3]);

    camToRobot = cam.robotToCam.inverse();
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
    cameraToTagTransform = null;
    deviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    if (confident) {

      // Get data from network tables (including timestamp)
      TimestampedDoubleArray data = cameraGlobalPoseSubscriber.getAtomic();

      // [0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw]
      double[] poseData = data.value;
      timestamp = data.timestamp;

      Pose3d cameraPose = poseFromTable(poseData);
      robotPose = cameraPose.transformBy(camToRobot);

      double[] deviationData = deviationSubscriber.get();
      deviation = deviationFromTable(deviationData);
    }

    if (tagCount > 0) {
      // [0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw]
      double[] transformData = tagToCameraTransformSubscriber.get();
      cameraToTagTransform = transformFromTable(transformData);
    }
  }

  private Pose3d poseFromTable(double[] data) {
    return new Pose3d(new Translation3d(data[0], data[1], data[2]),
        new Rotation3d(new Quaternion(data[3], data[4], data[5], data[6])));
  }

  private Transform3d transformFromTable(double[] data) {
    return new Transform3d(new Translation3d(data[0], data[1], data[2]),
        new Rotation3d(new Quaternion(data[3], data[4], data[5], data[6])));
  }

  private Matrix<N3, N1> deviationFromTable(double[] data) {
    return VecBuilder.fill(data[0], data[1], data[2]);
  }
}
