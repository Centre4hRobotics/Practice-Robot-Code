package frc.robot.subsystems;

<<<<<<< HEAD
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.MotorConstants;

public class KrakenSwerveModule extends SwerveModuleBase{
    
    private static MotorType[] motorTypes = {MotorType.TALON, MotorType.TALON};

    private TalonFXConfiguration driveConfig, turnConfig;

    public KrakenSwerveModule(int[] ids, int rotationEncoderID, String name) throws Exception{ 
        super(ids, motorTypes, rotationEncoderID, name);
        configureDrive();
        configureTurn();
    }

    public void configureDrive() {

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = MotorConstants.driveStatorCurrentLimit.abs(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.driveSupplyCurrentLimit.abs(Amps);

        driveConfig.Slot0.kP = MotorConstants.driveP;
        driveConfig.Slot0.kI = MotorConstants.driveI;
        driveConfig.Slot0.kD = MotorConstants.driveD;
        driveConfig.Slot0.kV = MotorConstants.driveKV;

        try {
            configure(driveConfig, "Driving Motor");
        } catch (Exception e) {
           e.printStackTrace();
        }
    }

    public void configureTurn() {

        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = MotorConstants.turnStatorCurrentLimit.abs(Amps);
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.turnSupplyCurrentLimit.abs(Amps);

        turnConfig.Slot0.kP = MotorConstants.turnP;
        turnConfig.Slot0.kI = MotorConstants.turnI;
        turnConfig.Slot0.kD = MotorConstants.turnD;
        turnConfig.Slot0.kV = MotorConstants.turnKV;

        try {
            configure(turnConfig, "Turning Motor");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

=======
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;;

public class KrakenSwerveModule extends SwerveModuleBase {
  private TalonFX drivingMotor;
  private TalonFX turningMotor;

  // what is applied to the motors
  private TalonFXConfiguration drivingConfiguration;
  private TalonFXConfiguration turningConfiguration;

  // stores configuration for the motor (technically not needed :))
  private TalonFXConfigurator drivingConfigurator;
  private TalonFXConfigurator turningConfigurator;

  private final VelocityVoltage driveVelocity = new VelocityVoltage(0.0);
  private final PositionVoltage anglePosition = new PositionVoltage(0.0);

  /**
   * Construct this swerve module
   *
   * @param driveDeviceId
   * @param turnDeviceId
   * @param absoluteEncoderId
   * @param absoluteEncoderAngleOffset
   * @param name
   * @param inverted
   */
  public KrakenSwerveModule(int driveDeviceId, int turnDeviceId, int absoluteEncoderId,
      double absoluteEncoderAngleOffset, String name, boolean inverted) {
    super(absoluteEncoderId, absoluteEncoderAngleOffset, name);

    drivingMotor = new TalonFX(driveDeviceId);
    turningMotor = new TalonFX(turnDeviceId);

    drivingConfiguration = new TalonFXConfiguration();
    turningConfiguration = new TalonFXConfiguration();

    drivingConfigurator = drivingMotor.getConfigurator();
    turningConfigurator = turningMotor.getConfigurator();

    configDriveMotor();
    configTurnMotor(inverted);

    resetDesiredState();

    syncEncoder();
  }

  @Override
  public void setPIDReference(SwerveModuleState swerveModuleState) {
    driveVelocity.Velocity =
        MPSToRPS(swerveModuleState.speedMetersPerSecond, RobotConstants.wheelCircumference);
    drivingMotor.setControl(driveVelocity);
    turningMotor.setControl(anglePosition.withPosition(swerveModuleState.angle.getRotations()));
  }

  public void setSlot(int slot) {
    driveVelocity.Slot = slot;
  }

  @Override
  public void setDriveSpeed(double speed) {
    drivingMotor.set(speed);
  }

  @Override
  public void setDriveVoltage(double voltage) {
    drivingMotor.setVoltage(voltage);
  }

  @Override
  public void setRotationSpeed(double speed) {
    turningMotor.set(speed);
  }

  @Override
  public void setRelativeRotationPosition(double position) {
    turningMotor.setPosition(position / 2 / Math.PI);
  }

  @Override
  public double getDriveVelocity() {
    return RPSToMPS(drivingMotor.getVelocity().getValueAsDouble(),
        RobotConstants.wheelCircumference);
  }

  @Override
  public double getDrivePosition() {
    return rotationsToMeters(drivingMotor.getPosition().getValueAsDouble(),
        RobotConstants.wheelCircumference);
  }

  @Override
  public double getRelativeRotationPositionRad() {
    return correctAngle(turningMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI);
  }

  /**
   * Configure the drive motor.
   */
  private void configDriveMotor() {
    drivingConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    drivingConfiguration.MotorOutput.Inverted = MotorConstants.inverted;
    drivingConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    drivingConfiguration.CurrentLimits.StatorCurrentLimit = MotorConstants.driveCurrentThreshold;
    drivingConfiguration.CurrentLimits.SupplyCurrentLowerTime = MotorConstants.driveTimeThreshold;

    drivingConfiguration.Feedback.SensorToMechanismRatio = RobotConstants.driveGearRatio;

    drivingConfiguration.Slot0.kP = MotorConstants.drivingP;
    drivingConfiguration.Slot0.kI = MotorConstants.drivingI;
    drivingConfiguration.Slot0.kD = MotorConstants.drivingD;
    drivingConfiguration.Slot0.kV = MotorConstants.drivingFF;
    // drivingConfiguration.Slot0.kS = MotorConstants.drivingS;

    drivingConfiguration.Slot1.kP = MotorConstants.bullyP;
    drivingConfiguration.Slot1.kI = MotorConstants.bullyI;
    drivingConfiguration.Slot1.kD = MotorConstants.bullyD;
    drivingConfiguration.Slot1.kV = MotorConstants.drivingFF;

    drivingConfigurator.apply(drivingConfiguration);
  }

  /**
   * Configure the turning motor.
   *
   * @param inverted Invert motor or not
   */
  private void configTurnMotor(boolean inverted) {
    turningConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (inverted) {
      drivingConfiguration.MotorOutput.Inverted = MotorConstants.inverted;
    } else {
      drivingConfiguration.MotorOutput.Inverted = MotorConstants.not_inverted;
    }
    turningConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    turningConfiguration.CurrentLimits.StatorCurrentLimit = MotorConstants.steerCurrentThreshold;
    turningConfiguration.CurrentLimits.SupplyCurrentLowerTime = MotorConstants.steerTimeThreshold;

    turningConfiguration.Feedback.SensorToMechanismRatio = RobotConstants.steerGearRatio;
    turningConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

    turningConfiguration.Slot0.kP = MotorConstants.wheelTurningP;
    turningConfiguration.Slot0.kI = MotorConstants.wheelTurningI;
    turningConfiguration.Slot0.kD = MotorConstants.wheelTurningD;

    turningConfiguration.Slot1.kP = MotorConstants.bullyTurnP;
    turningConfiguration.Slot1.kI = MotorConstants.bullyTurnI;
    turningConfiguration.Slot1.kD = MotorConstants.bullyTurnD;

    turningConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turningConfigurator.apply(turningConfiguration);
  }

  public void setValues(double... values) {}

  @Override
  public void setTurnEncoderPIDTarget(double target) {
    anglePosition.Position = target;
    turningMotor.setControl(anglePosition);
  }

  /**
   * Convert wheel speed in rotations per second to meters per second {@summary} test
   *
   * @param wheelRPS The rotational velocity of the wheel in rotations per second.
   * @param circumference The circumference of the wheel in inches.
   * @return The velocity in meters per second.
   */
  private double RPSToMPS(double wheelRPS, double circumference) {
    double wheelMPS = wheelRPS * (circumference * 2.54 / 100);
    return wheelMPS;
  }

  /**
   * Convert speed in meters per second to the rotations per second of a wheel.
   *
   * @param wheelMPS The speed in meters per second.
   * @param circumference The circumference of the wheel in inches.
   * @return The rotational velocity of the wheel in rotations per second.
   */
  private double MPSToRPS(double wheelMPS, double circumference) {
    double wheelRPS = wheelMPS / (circumference * 2.54 / 100);
    return wheelRPS;
  }

  /**
   * Convert rotations of a wheel to meters travelled.
   * 
   * @param wheelRotations The number of rotations of the wheel.
   * @param circumference The circumference of the wheel, in inches.
   * @return The distance travelled by the wheel in meters.
   */
  private double rotationsToMeters(double wheelRotations, double circumference) {
    double wheelMeters = wheelRotations * (circumference * 25.4 / 1000.0);
    return wheelMeters;
  }
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1
}
