package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Vision extends SubsystemBase {

    protected Pose3d robotPose;
    protected long tagCount;
    protected boolean confident;
    protected double timestamp;
    protected long[] tagIDs;

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

    public long[] getTagIDs() {
        return tagIDs;
    }

    /**
     * @return Whether the camera is confident about its pose
     */
    public boolean isConfident() {
        return confident;
    }

    /**
     * @return When the camera saw the tags, in FPGA time
     */
    public double getTimestamp() {
        return timestamp;
    }

    /**
     * Calculate the actual pose.
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

        if (robotPose != null)
        {
            visionTable.getEntry("Pose X").setValue(robotPose.getX());
            visionTable.getEntry("Pose Y").setValue(robotPose.getY());
            visionTable.getEntry("Pose Theta").setValue(robotPose.getRotation().getZ());

            fusedPosePublisher.set(robotPose.toPose2d());
        }
    }
}