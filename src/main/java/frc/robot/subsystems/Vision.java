package frc.robot.subsystems;

<<<<<<< HEAD
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    public Pose2d getPose() {
        //will imp
        return null;
    }

    public double getTimeStamp() {
        //will imp
        return 0;
    }

=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Vision extends SubsystemBase {

  protected Pose3d robotPose = new Pose3d();
  protected long tagCount = 0;
  protected boolean confident = false;
  protected double timestamp = -1;
  protected long[] tagIDs = new long[] {};

  protected long bestTagID = 0;
  protected Transform3d tagToCameraTransform = new Transform3d();

  protected NetworkTable visionTable;
  protected StructPublisher<Pose2d> fusedPosePublisher;

  /**
   * @return The pose of the robot on the field
   */
  public Pose3d getRobotPose() {
    return robotPose;
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
   * @return Get the most centered tag's ID.
   */
  public long getCenteredTag() {
    return bestTagID;
  }

  /**
   * @return Get the transform from the most centered tag to the camera.
   * @see #getCenteredTag()
   */
  public Transform3d getTagToCamera() {
    return tagToCameraTransform;
  }

  /**
   * Calculate the actual pose. MUST be implemented by subclasses.
   */
  abstract protected void createPose();

  public Vision() {
    visionTable = NetworkTableInstance.getDefault().getTable("AprilTag Vision");

    fusedPosePublisher = visionTable.getStructTopic("Fused Pose", Pose2d.struct).publish();
  }

  @Override
  public void periodic() {
    createPose();

    visionTable.getEntry("TagCount").setValue(tagCount);
    visionTable.getEntry("Tag IDs").setValue(tagIDs);
    visionTable.getEntry("isConfident").setValue(confident);

    if (robotPose != null) {
      visionTable.getEntry("Pose X").setValue(robotPose.getX());
      visionTable.getEntry("Pose Y").setValue(robotPose.getY());
      visionTable.getEntry("Pose Theta").setValue(robotPose.getRotation().getZ());

      fusedPosePublisher.set(robotPose.toPose2d());
    }
  }
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1
}
