//Taliesin Halferty -- May 16th 2026

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Revolutions;

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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FlexyMech extends SubsystemBase{

    private TriMap<Integer, String, FlexyBrushlessController> map = new TriMap<>();
    
    public FlexyMech(int[] ids, MotorType[] types) {

        MotorType motorType = MotorType.TALON;
        FlexyBrushlessController controller;

        for(int i = 0; i<ids.length; i++) {
            motorType = MotorType.TALON;
            if(i<types.length)
                motorType = types[i];
            controller = new FlexyBrushlessController(ids[i], motorType);
            map.put(ids[i], "Motor " + ids[i] + ": " + motorType.name(), controller);
        }
    }

    public void setNameList(String[] names, int[] ids) {
        if(names.length!=ids.length)
            return;
        FlexyBrushlessController temp;
        for(int i = 0; i<ids.length; i++) {
            temp = map.getT3FromT1(ids[i]);
            map.removeT1(ids[i]);
            map.put(ids[i], names[i], temp);
        }
    }

    protected void configure(int id, TalonFXConfiguration talonConfig, SparkMaxConfig sparkConfig) {
        if(map.contains(id)) {
            switch (map.getT3FromT1(id).getType()) {
                case SPARK:
                    map.getT3FromT1(id).config(sparkConfig);
                    break;
                case TALON:
                default:
                    map.getT3FromT1(id).config(talonConfig);
                    break;
            }
        }
    }

    protected void configure(int[] ids, TalonFXConfiguration talonConfig, SparkMaxConfig sparkConfig) {
        for(int id : ids)
            configure(id, talonConfig, sparkConfig);
    }

    protected void configure(String name, TalonFXConfiguration talonConfig, SparkMaxConfig sparkConfig) {
        if(map.contains(name))
            configure(map.getT1FromT2(name), talonConfig, sparkConfig);
    }

    protected void configure(String[] names, TalonFXConfiguration talonConfig, SparkMaxConfig sparkConfig) {
        for(String name : names)
            configure(name, talonConfig, sparkConfig);
    }

    protected void invert(int id, boolean inverted) {
        if(map.contains(id))
            map.getT3FromT1(id).setInverted(inverted);
    }

    protected void invert(int[] ids, boolean inverted) {
        for(int id : ids)
            invert(id, inverted);
    }

    protected void invert(String name, boolean inverted) {
        if(map.contains(name))
            invert(map.getT1FromT2(name), inverted);
    }

    protected void invert(String[] names, boolean inverted) {
        for(String name : names)
            invert(name, inverted);
    }

    protected void makeFollowers(int followerId, int leaderId, boolean alignment) {
        if(map.contains(followerId)&&map.contains(leaderId))
            map.getT3FromT1(followerId).makeFollower(map.getT3FromT1(leaderId), alignment);
    }

    protected void makeFollowers(String followerName, String leaderName, boolean alignment) {
        if(map.contains(followerName)&&map.contains(leaderName))
            makeFollowers(map.getT1FromT2(followerName), map.getT1FromT2(leaderName), alignment);
    }

    public void spin(int id, Voltage volts) {
        if(map.contains(id))
            map.getT3FromT1(id).spin(volts);
    }

    public void spin(int[] ids, Voltage volts) {
        for(int id : ids)
            spin(id, volts);
    }

    public void spin(String name, Voltage volts) {
        if(map.contains(name))
            spin(map.getT1FromT2(name), volts);
    }

    public void spin(String[] names, Voltage volts) {
        for(String name : names)
            spin(name, volts);
    }

    public void spin(int id, AngularVelocity rps) {
        if(map.contains(id))
            map.getT3FromT1(id).spin(rps);
    }

    public void spin(int[] ids, AngularVelocity rps) {
        for(int id : ids)
            spin(id, rps);
    }

    public void spin(String name, AngularVelocity rps) {
        if(map.contains(name))
            spin(map.getT1FromT2(name), rps);
    }

    public void spin(String[] names, AngularVelocity rps) {
        for(String name : names)
            spin(name, rps);
    }

    public void spin(int id, Angle angle) {
        if(map.contains(id))
            map.getT3FromT1(id).spin(angle);
    }

    public void spin(int[] ids, Angle angle) {
        for(int id : ids)
            spin(id, angle);
    }

    public void spin(String name, Angle angle) {
        if(map.contains(name))
            spin(map.getT1FromT2(name), angle);
    }

    public void spin(String[] names, Angle angle) {
        for(String name : names)
            spin(name, angle);
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
            if(_type == MotorType.TALON) {
                _talonConfig = config;
                _talon.getConfigurator().apply(_talonConfig);
            }
        }

        public void config(SparkMaxConfig config) {
            if(_type == MotorType.SPARK) {
                _sparkConfig = config;
                _spark.configure(_sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }
        }

        public void setInverted(boolean inverted) {
            switch (_type) {
                case SPARK:
                    _sparkConfig.inverted(inverted);
                    _spark.configure(_sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                    break;
                case TALON:
                default:
                    if(inverted)
                        _talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                    else
                        _talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                    _talon.getConfigurator().apply(_talonConfig);
                    break;
            }
        }

        public void makeFollower(TalonFX other, MotorAlignmentValue alignment) {
            if(_type == MotorType.TALON)
                _talon.setControl(new Follower(other.getDeviceID(), alignment));
        }

        public void makeFollower(SparkMax other, boolean alignment) {
            if(_type == MotorType.SPARK) {
                _sparkConfig.follow(other, alignment);
                _spark.configure(_sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }
        }

        public void makeFollower(FlexyBrushlessController other, boolean alignment) {
            if(_type == other.getType()) {
                switch (_type) {
                    case SPARK:
                        makeFollower(other.getSparkMax(), alignment);
                        break;
                    case TALON:
                    default:
                        MotorAlignmentValue alignmentValue = MotorAlignmentValue.Opposed;
                        if(alignment)
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
            switch(_type) {
                case SPARK:
                    _spark.getClosedLoopController().setSetpoint(angle.abs(Revolutions), ControlType.kPosition);
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
            if(_type == MotorType.TALON)
                return _talon;
            return null;
        }

        public SparkMax getSparkMax() {
            if(_type == MotorType.SPARK)
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

    public enum MotorType {SPARK, TALON}

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
            return hash12.containsKey(o)||hash21.containsKey(o)||hash31.containsKey(o);
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
