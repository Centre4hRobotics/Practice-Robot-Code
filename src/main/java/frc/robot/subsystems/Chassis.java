package frc.robot.subsystems;

<<<<<<< HEAD
<<<<<<< HEAD
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Chassis extends SubsystemBase{
    
    private SwerveModuleBase[] modules;

    private SwerveDriveKinematics kinematics;

    private SlewRateLimiter slewRateLimiterLinear = new SlewRateLimiter(MotorConstants.maxLinearSlewRate.abs(MetersPerSecondPerSecond)), 
        slewRateLimiterTheta = new SlewRateLimiter(MotorConstants.maxAngularSlewRate.abs(RotationsPerSecondPerSecond));
    
    
    public Chassis(int[] driveIDs, int[] turnIDs, int[] encoderIDs) throws Exception {

        if(driveIDs.length!=turnIDs.length||encoderIDs.length!=driveIDs.length||encoderIDs.length!=turnIDs.length)
            throw new Exception("Drive ID List, Turn ID List, and Encoder ID List are not all of the same length");

        String[] names = {"Front Right Module", "Back Right Module", "Front Left Module", "Back Left Module"};
        int[] temp = new int[2];
        for(int i = 0; i<driveIDs.length; i++) {
            temp[0] = driveIDs[i];
            temp[1] = turnIDs[i];
            modules[i] = new KrakenSwerveModule(temp, encoderIDs[i], names[i]);
        }

        double moduleRadius = MotorConstants.moduleDistance.abs(Meter)/2;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(moduleRadius, moduleRadius),
            new Translation2d(moduleRadius, -moduleRadius),
            new Translation2d(-moduleRadius, moduleRadius),
            new Translation2d(-moduleRadius, -moduleRadius)
        );
    }

    public void setDesiredStates(LinearVelocity linearVelocity, Angle driveAngle, AngularVelocity angularVelocity, Pose2d currentPose) {

        double maxLinear = MotorConstants.maxDrivingVelocity.abs(MetersPerSecond);
        linearVelocity = Units.MetersPerSecond.of(slewRateLimiterLinear.calculate(Math.min(Math.max(linearVelocity.abs(MetersPerSecond), -maxLinear), maxLinear)));
        
        double maxAngular = MotorConstants.maxDrivingAngularVelocity.abs(RotationsPerSecond);
        angularVelocity = Units.RevolutionsPerSecond.of(slewRateLimiterTheta.calculate(Math.min(Math.max(angularVelocity.abs(RevolutionsPerSecond), -maxAngular), maxAngular)));

        ChassisSpeeds commandedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity.times(Math.cos(driveAngle.abs(Radians))), linearVelocity.times(Math.cos(driveAngle.abs(Radians))), angularVelocity, currentPose.getRotation());

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(commandedSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MotorConstants.maxVelocity);

        for(int i = 0; i<moduleStates.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] output = new SwerveModulePosition[modules.length];
        for(int i = 0; i<output.length; i++) {
            output[i] = modules[i].getPosition();
        }
        return output;
    }
    
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
=======
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1
=======
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1

}
