package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Vision extends SubsystemBase {

    protected Pose2d robotPose;

    protected double timestamp;

    protected boolean confident;

    protected long[] tagIDs;
    protected int tagCount;
    protected int bestTagID;

    protected Transform3d tagToCameraTransform;

    protected NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");

    public abstract void createPose();

    public double getTimeStamp() {
        return timestamp;
    }

    public Pose2d getPose() {
        return robotPose;
    }

    public void periodic() {
        createPose();
    }

}
