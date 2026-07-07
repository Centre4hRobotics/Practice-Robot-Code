// Taliesin Halferty -- May 16th 2026

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.HashMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FlexyMech extends SubsystemBase {

  private TriMap<Integer, String, FlexyBrushlessController> map = new TriMap<>();

  private String name;

  private DoublePublisher[] velPubList, angPubList;
  private int[] ids;

  public FlexyMech(int[] ids, MotorType[] types, String name) {

    this.name = name;
    this.ids = ids;

    MotorType motorType = MotorType.TALON;
    FlexyBrushlessController controller;

    for (int i = 0; i < ids.length; i++) {
      motorType = MotorType.TALON;
      if (i < types.length)
        motorType = types[i];
      controller = new FlexyBrushlessController(ids[i], motorType);
      map.put(ids[i], "Motor " + ids[i] + ": " + motorType.name(), controller);
    }

    velPubList = new DoublePublisher[ids.length];
    angPubList = new DoublePublisher[ids.length];

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    for (int i = 0; i < ids.length; i++) {
      velPubList[i] =
          nt.getTable(name).getDoubleTopic("Motor ID: " + ids[i] + ", Angular Velocity").publish();
      angPubList[i] = nt.getTable(name).getDoubleTopic("Motor ID: " + ids[i] + ", Angle").publish();
    }
  }

  public void setName(String name) {
    this.name = name;
  }

  public void setNameList(String[] names, int[] ids) {
    if (names.length != ids.length)
      return;
    FlexyBrushlessController temp;
    for (int i = 0; i < ids.length; i++) {
      temp = map.getT3FromT1(ids[i]);
      map.removeT1(ids[i]);
      map.put(ids[i], names[i], temp);
    }
  }

  protected void configure(SparkMaxConfig config, int... ids) throws Exception {
    for (int id : ids) {
      if (map.getT3FromT1(id).getType() != MotorType.SPARK)
        throw new Exception("Must Configure Talon with TalonFXConfiguration");
      map.getT3FromT1(id).config(config);
    }
  }

  protected void configure(TalonFXConfiguration config, int... ids) throws Exception {
    for (int id : ids) {
      if (map.getT3FromT1(id).getType() != MotorType.TALON)
        throw new Exception("Must Configure SparkMax with SparkMaxConfig");
      map.getT3FromT1(id).config(config);
    }
  }

  protected void configure(TalonFXConfiguration config, String... names) {
    for (String name : names) {
      try {
        configure(config, map.getT1FromT2(name));
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  protected void configure(SparkMaxConfig config, String... names) {
    for (String name : names) {
      try {
        configure(config, map.getT1FromT2(name));
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  protected void makeFollower(int followerId, int leaderId, boolean alignment) {
    try {
      map.getT3FromT1(followerId).makeFollower(map.getT3FromT1(leaderId), alignment);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  protected void makeFollower(String followerName, String leaderName, boolean alignment) {
    try {
      makeFollower(map.getT1FromT2(followerName), map.getT1FromT2(leaderName), alignment);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void spin(Voltage volts, int... ids) {
    for (int id : ids) {
      try {
        map.getT3FromT1(id).spin(volts);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public void spin(Voltage volts, String... names) {
    for (String name : names) {
      try {
        map.getT3FromT2(name).spin(volts);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public void spin(AngularVelocity rps, int... ids) {
    for (int id : ids) {
      try {
        map.getT3FromT1(id).spin(rps);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public void spin(AngularVelocity rps, String... names) {
    for (String name : names) {
      try {
        map.getT3FromT2(name).spin(rps);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public void spin(Angle angle, int... ids) {
    for (int id : ids) {
      try {
        map.getT3FromT1(id).spin(angle);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public void spin(Angle angle, String... names) {
    for (String name : names) {
      try {
        map.getT3FromT2(name).spin(angle);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public AngularVelocity getRPS(int id) {
    try {
      return map.getT3FromT1(id).getRPS();
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  public AngularVelocity[] getRPS(int... ids) {
    AngularVelocity[] output = new AngularVelocity[ids.length];
    for (int i = 0; i < ids.length; i++)
      output[i] = getRPS(ids[i]);
    return output;
  }

  public AngularVelocity getRPS(String name) {
    try {
      return map.getT3FromT2(name).getRPS();
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  public AngularVelocity[] getRPS(String... names) {
    AngularVelocity[] output = new AngularVelocity[names.length];
    for (int i = 0; i < names.length; i++)
      output[i] = getRPS(names[i]);
    return output;
  }

  public Angle getAngle(int id) {
    try {
      return map.getT3FromT1(id).getAngle();
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  public Angle[] getAngle(int... ids) {
    Angle[] output = new Angle[ids.length];
    for (int i = 0; i < ids.length; i++)
      output[i] = getAngle(ids[i]);
    return output;
  }

  public Angle getAngle(String name) {
    try {
      return map.getT3FromT2(name).getAngle();
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  public Angle[] getAngle(String... names) {
    Angle[] output = new Angle[names.length];
    for (int i = 0; i < names.length; i++)
      output[i] = getAngle(names[i]);
    return output;
  }

  public TriMap<Integer, String, FlexyBrushlessController> getTriMap() {
    return map;
  }

  public String getName() {
    return name;
  }

  public void log() {
    for (int i = 0; i < ids.length; i++) {
      velPubList[i].set(getRPS(ids[i]).in(RotationsPerSecond));
      angPubList[i].set(getAngle(ids[i]).in(Degrees));
    }
  }

  public void periodic() {
    log();
  }

  public class FlexyBrushlessController {

    private TalonFX _talon;
    private TalonFXConfiguration _talonConfig;

    private SparkMax _spark;
    private SparkMaxConfig _sparkConfig;

    private final MotorType _type;

    public FlexyBrushlessController(int id, MotorType type) {
      _type = type;
      switch (_type) {
        case SPARK:
          _spark = new SparkMax(id, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
          break;
        case TALON:
        default:
          _talon = new TalonFX(id);
          break;
      }
    }

    public void config(TalonFXConfiguration config) {
      if (_type == MotorType.TALON) {
        _talonConfig = config;
        _talon.getConfigurator().apply(_talonConfig);
      }
    }

    public void config(SparkMaxConfig config) {
      if (_type == MotorType.SPARK) {
        _sparkConfig = config;
        _spark.configure(_sparkConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      }
    }

    public void setInverted(boolean inverted) {
      switch (_type) {
        case SPARK:
          _sparkConfig.inverted(inverted);
          _spark.configure(_sparkConfig, ResetMode.kResetSafeParameters,
              PersistMode.kPersistParameters);
          break;
        case TALON:
        default:
          if (inverted)
            _talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          else
            _talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          _talon.getConfigurator().apply(_talonConfig);
          break;
      }
    }

    public void makeFollower(TalonFX other, MotorAlignmentValue alignment) {
      if (_type == MotorType.TALON)
        _talon.setControl(new Follower(other.getDeviceID(), alignment));
    }

    public void makeFollower(SparkMax other, boolean alignment) {
      if (_type == MotorType.SPARK) {
        _sparkConfig.follow(other, alignment);
        _spark.configure(_sparkConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      }
    }

    public void makeFollower(FlexyBrushlessController other, boolean alignment) {
      if (_type == other.getType()) {
        switch (_type) {
          case SPARK:
            makeFollower(other.getSparkMax(), alignment);
            break;
          case TALON:
          default:
            MotorAlignmentValue alignmentValue = MotorAlignmentValue.Opposed;
            if (alignment)
              alignmentValue = MotorAlignmentValue.Aligned;
            makeFollower(other.getTalon(), alignmentValue);
            break;
        }
      }
    }

    public void spin(Voltage volts) {
      switch (_type) {
        case SPARK:
          _spark.setVoltage(volts.in(Units.Volts));
          break;
        case TALON:
        default:
          _talon.setVoltage(volts.in(Units.Volts));
          break;
      }
    }

    public void spin(AngularVelocity rps) {
      switch (_type) {
        case SPARK:
          _spark.getClosedLoopController().setSetpoint(rps.in(Units.RPM), ControlType.kVelocity);
          break;
        case TALON:
        default:
          _talon.setControl(new VelocityVoltage(rps));
          break;
      }
    }

    public void spin(Angle angle) {
      switch (_type) {
        case SPARK:
          _spark.getClosedLoopController().setSetpoint(angle.abs(Revolutions),
              ControlType.kPosition);
          break;
        case TALON:
        default:
          _talon.setControl(new PositionDutyCycle(angle));
          break;
      }
    }

    public Angle getAngle() {
      switch (_type) {
        case SPARK:
          return Units.Rotations.of(_spark.getAbsoluteEncoder().getPosition());
        case TALON:
        default:
          return Units.Rotations.of(_talon.getPosition().getValueAsDouble());
      }
    }

    public AngularVelocity getRPS() {
      switch (_type) {
        case SPARK:
          return Units.RPM.of(_spark.getAbsoluteEncoder().getVelocity());
        case TALON:
        default:
          return Units.RPM.of(_talon.getVelocity().getValueAsDouble());
      }
    }

    public TalonFX getTalon() {
      if (_type == MotorType.TALON)
        return _talon;
      return null;
    }

    public SparkMax getSparkMax() {
      if (_type == MotorType.SPARK)
        return _spark;
      return null;
    }

    public int getDeviceID() {
      switch (_type) {
        case SPARK:
          return _spark.getDeviceId();
        case TALON:
        default:
          return _talon.getDeviceID();
      }
    }

    public MotorType getType() {
      return _type;
    }
  }

  public enum MotorType {
    SPARK, TALON
  }

  public class TriMap<T1, T2, T3> {
    private HashMap<T1, T2> hash12;
    private HashMap<T2, T3> hash23;
    private HashMap<T3, T1> hash31;
    private HashMap<T1, T3> hash13;
    private HashMap<T3, T2> hash32;
    private HashMap<T2, T1> hash21;

    public TriMap() {
      hash12 = new HashMap<>();
      hash23 = new HashMap<>();
      hash31 = new HashMap<>();
      hash13 = new HashMap<>();
      hash32 = new HashMap<>();
      hash21 = new HashMap<>();
    }

    public boolean contains(Object o) {
      return hash12.containsKey(o) || hash21.containsKey(o) || hash31.containsKey(o);
    }

    public void put(T1 t1, T2 t2, T3 t3) {
      hash12.put(t1, t2);
      hash21.put(t2, t1);
      hash13.put(t1, t3);
      hash31.put(t3, t1);
      hash23.put(t2, t3);
      hash32.put(t3, t2);
    }

    public T1 getT1FromT2(T2 t2) {
      return hash21.get(t2);
    }

    public T1 getT1FromT3(T3 t3) {
      return hash31.get(t3);
    }

    public T2 getT2FromT1(T1 t1) {
      return hash12.get(t1);
    }

    public T2 getT2FromT3(T3 t3) {
      return hash32.get(t3);
    }

    public T3 getT3FromT1(T1 t1) {
      return hash13.get(t1);
    }

    public T3 getT3FromT2(T2 t2) {
      return hash23.get(t2);
    }

    public void removeT1(T1 t1) {
      T2 t2 = hash12.get(t1);
      T3 t3 = hash13.get(t1);

      hash12.remove(t1);
      hash13.remove(t1);
      hash21.remove(t2);
      hash23.remove(t2);
      hash31.remove(t3);
      hash32.remove(t3);
    }

    public void removeT2(T2 t2) {
      T1 t1 = hash21.get(t2);
      T3 t3 = hash23.get(t2);

      hash12.remove(t1);
      hash13.remove(t1);
      hash21.remove(t2);
      hash23.remove(t2);
      hash31.remove(t3);
      hash32.remove(t3);
    }

    public void removeT3(T3 t3) {
      T1 t1 = hash31.get(t3);
      T2 t2 = hash32.get(t3);

      hash12.remove(t1);
      hash13.remove(t1);
      hash21.remove(t2);
      hash23.remove(t2);
      hash31.remove(t3);
      hash32.remove(t3);
    }
  }

}
