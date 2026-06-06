package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

abstract public class SwerveModuleBase {
  private CANcoder _absoluteTurningEncoder;
  private SwerveModuleState _desiredState;
  private double _absoluteEncoderAngleOffset;

  private String _name;

  public SwerveModuleBase(int absoluteEncoderId, double absoluteEncoderAngleOffset, String name) {
    _absoluteTurningEncoder = new CANcoder(absoluteEncoderId);
    _absoluteEncoderAngleOffset = absoluteEncoderAngleOffset;

    _name = name;
  }

  /**
   * Set desiredState variable to current position and zero speed
   */
  public void resetDesiredState() {
    _desiredState = new SwerveModuleState(0.0, new Rotation2d(getRelativeRotationPositionRad()));
  }

  /**
   * Set the desired state for PID control
   *
   * @param state SwerveModuleState
   */
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState correctedState = new SwerveModuleState();
    correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
    correctedState.angle = state.angle;
    correctedState.optimize(new Rotation2d(getRelativeRotationPositionRad()));

    setPIDReference(correctedState);

    _desiredState = state;
  }

  /**
   * Sync the relative encoder to the absolute encoder
   *
   * @return the position read from the absolute encoder
   */
  public double syncEncoder() {
    double absolutePos = getAbsoluteRotationPositionRad();

    setRelativeRotationPosition(absolutePos);

    return absolutePos;
  }

  public abstract void setSlot(int slot);


  /**
   * Gives a state to the PID controllers to reference
   *
   * @param swerveModuleState
   */
  public abstract void setPIDReference(SwerveModuleState swerveModuleState);


  /**
   * Sets the speed for the drive motor
   *
   * @param speed
   */
  public abstract void setDriveSpeed(double speed);

  /**
   * Sets the voltage of the drive motor
   *
   * @param voltage 0 to 12 volts
   */
  public abstract void setDriveVoltage(double voltage);

  /**
   * Sets the speed for the rotation motor
   *
   * @param speed
   */
  public abstract void setRotationSpeed(double speed);

  /**
   * Sets the position of the relative rotation encoder
   *
   * @param position the position in radians
   */
  public abstract void setRelativeRotationPosition(double position);

  /**
   * Sets the turn encoder PID target
   *
   * @param target The angle to pass to the PID
   */
  public abstract void setTurnEncoderPIDTarget(double target);

  /**
   * Get drive velocity from the driving encoder
   *
   * @return
   */
  public abstract double getDriveVelocity();

  /**
   * Get drive position from the driving encoder
   *
   * @return
   */
  public abstract double getDrivePosition();

  /**
   * Get relative rotation in radians
   *
   * @return
   */
  public abstract double getRelativeRotationPositionRad();

  /**
   * Get absolute rotation in rotations
   *
   * @return
   */
  public double getAbsoluteRotationPositionRotations() {
    return _absoluteTurningEncoder.getAbsolutePosition().getValueAsDouble()
        - (_absoluteEncoderAngleOffset / (2 * Math.PI));
  }

  /**
   * Get absolute rotation in radians
   */
  public double getAbsoluteRotationPositionRad() {
    double _rotation =
        _absoluteTurningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI
            - _absoluteEncoderAngleOffset;
    return correctAngle(_rotation);
  }

  /**
   * Get the current state of the module based on encoders
   *
   * @return
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(),
        new Rotation2d(getRelativeRotationPositionRad()));
  }

  /**
   * Get the current desired state
   *
   * @return
   */
  public SwerveModuleState getDesiredState() {
    return _desiredState;
  }

  /**
   * Gets the current rotation and driving position from encoders
   *
   * @return
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(),
        new Rotation2d(getRelativeRotationPositionRad()));
  }

  public double getAbsoluteEncoderAngleOffset() {
    return _absoluteEncoderAngleOffset;
  }

  public void setAbsoluteEncoderAngleOffset(double absoluteEncoderAngleOffset) {
    _absoluteEncoderAngleOffset = absoluteEncoderAngleOffset;
  }

  public CANcoder getAbsoluteTurningEncoder() {
    return _absoluteTurningEncoder;
  }


  /**
   * Forces {@code relativeAngle} to be between 0 and 2 PI
   *
   * @param relativeAngle relative angle from relative encoder on turning motor
   */
  protected double correctAngle(double relativeAngle) {
    double correctAngle = relativeAngle % (2 * Math.PI);
    if (correctAngle < 0) {
      return correctAngle + 2 * Math.PI;
    } else {
      return correctAngle;
    }
  }

  /**
   * Logs to the NetworkTable about the module
   */
  public void log() {
    // getLogEntry("Driving velocity").setDouble(getDriveVelocity());

    getLogEntry("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        .setString("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
    getLogEntry("@Relative encoder position").setDouble(getRelativeRotationPositionRad()); // * 180
                                                                                           // /
                                                                                           // Math.PI);
    getLogEntry("@Absolute encoder position").setDouble(getAbsoluteRotationPositionRad()); // * 180
                                                                                           // /
                                                                                           // Math.PI);


    // getLogEntry("Current state").setValue(getState());
    // getLogEntry("Desired state").setValue(_desiredState);

    getLogEntry("Desired state velocity").setDouble(_desiredState.speedMetersPerSecond);
    getLogEntry("Desired state angle").setDouble(_desiredState.angle.getDegrees());
    getLogEntry("Current state velocity").setDouble(getState().speedMetersPerSecond);
    getLogEntry("Current state angle").setDouble(getState().angle.getDegrees());
  }

  /**
   * Helper function to get the NetworkTableEntry for logs
   *
   * @param logName name of the item being logged
   * @return the entry to log
   */
  private NetworkTableEntry getLogEntry(String logName) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    return nt.getTable("SwerveModule").getEntry("Module " + _name + " " + logName);
  }
}
