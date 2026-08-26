package frc.robot.subsystems.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;


/**
 * Subclasses must set `robotPose`, `tagCount`, `tagIDs`, `confident`, `timestamp`, `bestTagID`, and
 * `cameraToTagTransform`
 */
public abstract class CameraBase {

  protected Pose3d robotPose = new Pose3d();
  protected long tagCount = 0;
  protected boolean confident = false;
  protected double timestamp = -1;
  protected long[] tagIDs = new long[] {};

  protected long bestTagID = 0;
  protected Transform3d cameraToTagTransform = new Transform3d();

  protected static NetworkTable visionTable;
  protected DoublePublisher globalPoseXPublisher;
  protected DoublePublisher globalPoseYPublisher;
  protected DoublePublisher globalPoseThetaPublisher;
  protected StructPublisher<Pose2d> fusedPosePublisher;
  protected double deviation;

  /**
   * @return The absolute pose of the robot on the field in 3D
   */
  public Pose3d getRobotPose3d() {
    return robotPose;
  }

  /**
   * @return The absolute pose of the robot on the field in 2D
   */
  public Pose2d getRobotPose2d() {
    return robotPose.toPose2d();
  }

  /**
   * @return The number of tags seen by the camera
   */
  public long getTagCount() {
    return tagCount;
  }

  /**
   * @return An array of all detected tag's IDs.
   */
  public long[] getTagIDs() {
    return tagIDs;
  }

  /**
   * @return Whether the camera is confident about its pose.
   */
  public boolean isConfident() {
    return confident;
  }

  /**
   * @return When the camera saw the tag(s), in FPGA time.
   */
  public double getTimestamp() {
    return timestamp;
  }

  /**
   * @return The most centered tag's ID.
   */
  public long getCenteredTag() {
    return bestTagID;
  }

  /**
   * @return The transform required to place the camera in the tag.
   * @see {@link #getCenteredTag() getCenteredTag} for which tag is selected.
   */
  public Transform3d getCameraToTag() {
    return cameraToTagTransform;
  }

  /**
   * Calculate the actual pose. MUST be implemented by subclasses.
   */
  abstract protected void createPose();

  public CameraBase(int index) {
    visionTable = NetworkTableInstance.getDefault().getTable("AprilTag Vision")
        .getSubTable("camera" + Integer.toString(index));

    fusedPosePublisher = visionTable.getStructTopic("Fused Pose", Pose2d.struct).publish();
    globalPoseXPublisher = visionTable.getDoubleTopic("Pose X").publish();
    globalPoseYPublisher = visionTable.getDoubleTopic("Pose Y").publish();
    globalPoseThetaPublisher = visionTable.getDoubleTopic("Pose Theta").publish();
  }

  public void update() {
    createPose();
    if (robotPose != null) {
      globalPoseXPublisher.set(robotPose.getX());
      globalPoseYPublisher.set(robotPose.getY());
      globalPoseThetaPublisher.set(robotPose.getRotation().getZ());

      fusedPosePublisher.set(robotPose.toPose2d());
    }
  }
}
