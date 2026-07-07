package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.MotorConstants;

public abstract class SwerveModuleBase extends FlexyMech {

    private CANcoder rotationEncoder;

    public SwerveModuleBase(int[] ids, MotorType[] motorTypes, int rotationEncoderID, String name)
            throws Exception {

        super(ids, motorTypes, name);

        if (ids.length != 2)
            throw new Exception("ID Array must be of length 2");
        if (motorTypes.length != 2)
            throw new Exception("Types Array must be of length 2");

        String[] names = {"Driving Motor", "Turning Motor"};

        setNameList(names, ids);

        rotationEncoder = new CANcoder(rotationEncoderID);

    }

    public Angle getSwerveAngle() {
        return rotationEncoder.getAbsolutePosition().getValue();
    }

    public LinearVelocity getSwerveVelocity() {
        try {
            return Units.MetersPerSecond.of(Math.PI * MotorConstants.wheelDiameter.abs(Meters)
                    * getRPS("Driving Motor").abs(RotationsPerSecond));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public Distance getSwerveDistance() {
        try {
            return MotorConstants.wheelDiameter.times(getAngle("Driving Motor").abs(Radians));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSwerveVelocity(),
                Rotation2d.fromRadians(getSwerveAngle().abs(Radians)));
    }

    public SwerveModulePosition getPosition() {
        try {
            return new SwerveModulePosition(getSwerveDistance().abs(Meters),
                    Rotation2d.fromRadians(getAngle("Turning Motor").abs(Radians)));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void setSwerveVelocity(LinearVelocity vel) {
        try {
            spin(Units.RevolutionsPerSecond.of(
                    vel.abs(MetersPerSecond) / Math.PI / MotorConstants.wheelDiameter.abs(Meters)),
                    "Driving Motor");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void setSwerveAngle(Angle angle) {
        try {
            spin(angle, "Turning Motor");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(Rotation2d.fromRadians(getSwerveAngle().abs(Radians)));
        setSwerveVelocity(Units.MetersPerSecond.of(state.speedMetersPerSecond));
        setSwerveAngle(state.angle.getMeasure());
    }

}
