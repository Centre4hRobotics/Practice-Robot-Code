// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorConstants {

    public static final Current driveStatorCurrentLimit = Units.Amps.of(45);
    public static final Current driveSupplyCurrentLimit = Units.Amps.of(45);
    public static final Current turnStatorCurrentLimit = Units.Amps.of(45);
    public static final Current turnSupplyCurrentLimit = Units.Amps.of(45);

    public static final double driveP = .1;
    public static final double driveI = .02;
    public static final double driveD = .01;
    public static final double driveKV = .73;

    public static final double turnP = .1;
    public static final double turnI = .02;
    public static final double turnD = .01;
    public static final double turnKV = .73;

    public static final LinearAcceleration maxLinearSlewRate = Units.MetersPerSecondPerSecond.of(1);
    public static final AngularAcceleration maxAngularSlewRate =
        Units.RotationsPerSecondPerSecond.of(1);

    public static final LinearVelocity maxVelocity = Units.MetersPerSecond.of(5);
    public static final AngularVelocity maxAngularVelocity = Units.RevolutionsPerSecond.of(1.5);
    public static final LinearVelocity maxDrivingVelocity = Units.MetersPerSecond.of(4.5);
    public static final AngularVelocity maxDrivingAngularVelocity =
        Units.RevolutionsPerSecond.of(1);

    public static final Distance wheelDiameter = Units.Inches.of(4);

    public static final Distance moduleDistance = Units.Inches.of(24);

    public static final int[] driveIDs = {1, 3, 5, 7};
    public static final int[] turnIDs = {2, 4, 6, 8};
    public static final int[] encoderIDs = {51, 52, 53, 54};
  }

  public static class VisionConstants {

    public static enum VisionType {
      COPROCESSOR, PHOTONVISION
    }

    public static class Camera {
      public final Transform3d robotToCam;

      public Camera(Transform3d robotToCam) {
        this.robotToCam = robotToCam;
      }
    }

    public static final Camera[] cameras =
        new Camera[] {new Camera(new Transform3d(0.23, -0.01, 0.3556, new Rotation3d()))};

    public static final VisionType visionType = VisionType.PHOTONVISION;

    public static final double jumpTolerance = .2;

    public static final List<Integer> bannedTags = null;
  }
}
