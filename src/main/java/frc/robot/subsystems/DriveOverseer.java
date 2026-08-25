package frc.robot.subsystems;

import java.util.List;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Vision.CameraBase;

public class DriveOverseer extends SubsystemBase {

    private Chassis chassis;
    private List<CameraBase> cameras;

    private Pigeon2 gyro;

    private SwerveDrivePoseEstimator poseEstimator;

    private StructPublisher<Pose2d> posePub;

    public DriveOverseer(Chassis chassis, List<CameraBase> cameras, int gyroID) {

        this.chassis = chassis;
        this.cameras = cameras;

        gyro = new Pigeon2(gyroID);

        poseEstimator = new SwerveDrivePoseEstimator(chassis.getKinematics(), gyro.getRotation2d(),
                chassis.getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        posePub = nt.getTable("Drive").getStructTopic("Pose", Pose2d.struct).publish();

    }

    public void drive(LinearVelocity linearVelocity, Angle driveAngle,
            AngularVelocity angularVelocity, double triggerSpeedUpInput) {
        linearVelocity = linearVelocity.plus(MotorConstants.maxVelocity
                .minus(MotorConstants.maxDrivingVelocity).times(triggerSpeedUpInput));
        angularVelocity = angularVelocity.plus(MotorConstants.maxAngularVelocity
                .minus(MotorConstants.maxDrivingAngularVelocity).times(triggerSpeedUpInput));
        chassis.setDesiredStates(linearVelocity, driveAngle, angularVelocity, getPose());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void periodic() {

        poseEstimator.update(gyro.getRotation2d(), chassis.getModulePositions());

        for (CameraBase cam : cameras) {
            Pose2d visionPose = cam.getRobotPose2d();
            if (visionPose != null)
                poseEstimator.addVisionMeasurement(visionPose, cam.getTimestamp());
        }

        posePub.set(getPose());
    }

}
