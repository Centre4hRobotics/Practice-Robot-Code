package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.Cameras.CameraBase;
import frc.robot.subsystems.Cameras.CameraCoprocessor;
import frc.robot.subsystems.Cameras.CameraPhoton;

public class Vision extends SubsystemBase {
  private List<CameraBase> cameras = new ArrayList<>();

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

  public void update(SwerveDrivePoseEstimator estimator) {
    for (CameraBase cam : cameras) {
      estimator.addVisionMeasurement(cam.getRobotPose2d(), cam.getTimestamp());
    }
  }

  @Override
  public void periodic() {
    for (CameraBase cam : cameras) {
      cam.update();
    }
  }
}
