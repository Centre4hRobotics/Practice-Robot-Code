// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.KrakenSwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class VisionConstants {
    public static final double jumpTolerance = 0.5; // in meters
    public static final Translation3d cameraOffset = new Translation3d(0, 0, 0);
    public static final Rotation3d cameraRotation = new Rotation3d(0, 0, 0);
    public static final Transform3d robotToCam = new Transform3d(cameraOffset, cameraRotation);
    public static final List<Integer> bannedTags =
        List.of(1, 6, 7, 12, 13, 14, 15, 16, 17, 22, 23, 28, 29, 30, 31, 32);
    public static final Transform3d camToRobot = robotToCam.inverse();
    public static final boolean usePhotonVision = false;
  }
  public static enum RobotType {
    PRACTICE, COMPETITION
  }

  public static RobotType robotType = RobotType.COMPETITION;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kFunctionControllerPort1 = 1;
    public static final int kFunctionControllerPort2 = 2;
  }

  public static class RobotConstants {
    public static final double wheelDiameter;

    public static final double maxAttainableSpeed; // arbitrary right now
    public static final double maxSlowDriveSpeed; // meters per second
    public static final double maxFastDriveSpeed;
    public static final double maxDriveAcceleration; // meters per second squared
    public static final double maxRotationSpeed; // radians per second
    public static final double fastRotationSpeed;
    public static final double maxRotationAcceleration; // guessed 2x max speed from PPLib examples

    public static final double driveGearRatio;
    public static final double steerGearRatio;


    public static final double maxSlewRate;
    public static final double robotLength = .7; // meters

    public static final double trackWidth;

    public static final double robotDriveBaseRadius; // distance from center to swerve module

    public static final double desiredFiringDistance = 2.5; // meters, will change
    public static final double posFiringDistanceBand = 2.7;
    public static final double negFiringDistanceBand = .75;
    public static final double distTrimRate = .05;


    static {
      switch (robotType) {
        case PRACTICE:
          wheelDiameter = 4.0;
          maxAttainableSpeed = 4.2;
          maxSlowDriveSpeed = 1.5;
          maxFastDriveSpeed = 4;
          maxDriveAcceleration = 1;
          maxRotationSpeed = 2 * Math.PI;
          fastRotationSpeed = 3 * Math.PI;
          maxRotationAcceleration = 3 * Math.PI;
          driveGearRatio = 6.2;
          steerGearRatio = 12.1;
          maxSlewRate = 20;
          trackWidth = Units.inchesToMeters(25);
          robotDriveBaseRadius = trackWidth * Math.sqrt(2) / 2;
          break;
        case COMPETITION:
        default:
          wheelDiameter = 4.0;
          maxAttainableSpeed = 5.15;
          maxSlowDriveSpeed = 2;
          maxFastDriveSpeed = 4;
          maxDriveAcceleration = 1.5;
          maxRotationSpeed = 2.25 * Math.PI;
          fastRotationSpeed = 3.5 * Math.PI;
          maxRotationAcceleration = 3 * Math.PI;
          driveGearRatio = 6.2;
          steerGearRatio = 12.1;
          maxSlewRate = 20;
          trackWidth = Units.inchesToMeters(20);
          robotDriveBaseRadius = trackWidth * Math.sqrt(2) / 2;
          break;
      }
    }
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    // top left, top right, bottom left, bottom right
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(new Translation2d(trackWidth / 2, trackWidth / 2), // ++
            new Translation2d(trackWidth / 2, -trackWidth / 2), // +-
            new Translation2d(-trackWidth / 2, trackWidth / 2), // -+
            new Translation2d(-trackWidth / 2, -trackWidth / 2) // --
        );
  }

  public static class GyroConstants {
    public static final int pigeonID = 11;

    public static final double mountPoseYawDegrees = 0;
    public static final double mountPosePitchDegrees = 0;
    public static final double mountPoseRollDegrees = 0;
  }

  public static class MotorConstants {

    // DELETE THIS LATER (SET IN PHOENIX TUNER)
    public static final double topLeftEncoderOffset = 0.0;
    public static final double topRightEncoderOffset = 0.0;
    public static final double bottomLeftEncoderOffset = 0.0;
    public static final double bottomRightEncoderOffset = 0.0;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue not_inverted = InvertedValue.CounterClockwise_Positive;

    public static KrakenSwerveModule[] getSwerveModules() {
      return new KrakenSwerveModule[] {
          new KrakenSwerveModule(3, 4, 51, topLeftEncoderOffset, "top left", false),
          new KrakenSwerveModule(5, 6, 52, topRightEncoderOffset, "top right", false),
          new KrakenSwerveModule(1, 2, 53, bottomLeftEncoderOffset, "bottom left", false),
          new KrakenSwerveModule(7, 8, 54, bottomRightEncoderOffset, "bottom right", false)};
    }

    // Turning refers to the wheels themselves in the swerve module.
    public static final double wheelTurningP;
    public static final double wheelTurningI;
    public static final double wheelTurningD;

    // Driving refers to teleoperate mode and the linear translation of the robot.
    public static final double drivingP;
    public static final double drivingI;
    public static final double drivingD;
    public static final double drivingS;
    public static final double drivingFF;

    public static final double bullyP;
    public static final double bullyI;
    public static final double bullyD;

    public static final double bullyTurnP;
    public static final double bullyTurnI;
    public static final double bullyTurnD;

    public static final double tanP;
    public static final double tanI;
    public static final double tanD;

    // o refers to the autonomous mode (contains both drive and wheel).
    public static final double autoDriveP;
    public static final double autoDriveI;
    public static final double autoDriveD;
    public static final double autoTurningP;
    public static final double autoTurningI;
    public static final double autoTurningD;

    // Heading refers to the heading of the entire robot as a whole.
    public static final double headingP;
    public static final double headingI;
    public static final double headingD;
    public static final double headingPeriod;

    // Creates the current/time thresholds
    // Current limits the draw of current
    // Time limits how long a motor can stall before decreasing power draw
    public static final double driveCurrentThreshold;
    public static final double driveTimeThreshold;
    public static final double steerCurrentThreshold;
    public static final double steerTimeThreshold;

    static {
      wheelTurningP = 20.0;
      wheelTurningI = 0.0;
      wheelTurningD = 0.0;
      drivingP = 0.0;
      drivingI = 0.0;
      drivingD = 0.0;
      drivingS = 0.9;
      drivingFF = 0.73;
      bullyP = 2.0;
      bullyI = 0.0;
      bullyD = 0.0;
      bullyTurnP = 2.0;
      bullyTurnI = 0.0;
      bullyTurnD = 0.0;

      switch (robotType) {
        case PRACTICE:
          autoDriveP = 5.0;
          autoDriveI = 0.3;
          autoDriveD = 0.0;
          autoTurningP = 4.0;
          autoTurningI = 0.3;
          autoTurningD = 0.0;
          headingP = 0.11;
          headingI = 0.02;
          headingD = 0.0;
          headingPeriod = 0.02;
          tanP = 0;
          tanI = 0;
          tanD = 0;
          driveCurrentThreshold = 50.0;
          driveTimeThreshold = 0.1;
          steerCurrentThreshold = 50.0;
          steerTimeThreshold = 0.1;
          break;
        case COMPETITION:
        default:
          autoDriveP = 2.5;
          autoDriveI = 0.1;
          autoDriveD = 0.0;
          autoTurningP = 2.0;
          autoTurningI = 0.2;
          autoTurningD = 0.0;
          headingP = 0.2;
          headingI = 0.02;
          headingD = 0.0;
          headingPeriod = 0.02;
          tanP = 1;
          tanI = 0;
          tanD = 0;
          driveCurrentThreshold = 50.0;
          driveTimeThreshold = 0.1;
          steerCurrentThreshold = 50.0;
          steerTimeThreshold = 0.1;
          break;
      }
    }

    public static final PIDController headingPIDController =
        new PIDController(headingP, headingI, headingD);
    public static final PIDController tangentPIDController = new PIDController(tanP, tanI, tanD);

    // HolonomicPathFollowerConfig, this should likely live in your Constants class
    // public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig = new
    // HolonomicPathFollowerConfig(
    // new PIDConstants(autoDriveP, autoDriveI,autoDriveD), // Translation PID constants
    // new PIDConstants(autoTurningP, autoTurningI, autoTurningI), // Rotation PID constants
    // RobotConstants.maxAttainableSpeed, // Max module speed, in m/s
    // RobotConstants.robotDriveBaseRadius, // Drive base radius in meters. Distance from robot
    // center to furthest module.
    // new ReplanningConfig(true, true) // can add thresholds for both values, see PPLib Java API
    // for info
    // );
  }
}
