package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.Cameras.CameraBase;
import frc.robot.subsystems.Cameras.CameraCoprocessor;
import frc.robot.subsystems.Cameras.CameraPhoton;

public class Vision extends SubsystemBase {
  private List<CameraBase> cameras = new ArrayList<>();
  private List<VisionPoseEstimate> estimates = new ArrayList<>();

  public static class VisionPoseEstimate {
    public final Pose2d robotPose2d;
    public final double timestamp;
    public final Matrix<N3, N1> standardDeviations;

    public VisionPoseEstimate(CameraBase camera) {
      robotPose2d = camera.getRobotPose2d();
      timestamp = camera.getTimestamp();
      standardDeviations = camera.getStandardDeviations();
    }
  }

  public Vision() {
    for (Camera cam : VisionConstants.cameras) {
      switch (cam.visionType) {
        case PHOTONVISION:
          cameras.add(new CameraPhoton(cam));
          break;
        case COPROCESSOR:
          cameras.add(new CameraCoprocessor(cam));
          break;
      }
    }
  }

  public List<VisionPoseEstimate> getPoseEstimates() {
    return estimates;
  }


  @Override
  public void periodic() {
    estimates.clear();
    for (CameraBase cam : cameras) {
      cam.update();
      estimates.add(new VisionPoseEstimate(cam));
    }
  }
}
