package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.ArrayList;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlexMech extends SubsystemBase{
    
    private ArrayList<PWMMotorController> motorList = new ArrayList<PWMMotorController>();
    private ArrayList<String> nameList = new ArrayList<String>();

    private MotorType[] _motorTypes;

    public FlexMech(int[] motorIDs, MotorType[] motorTypes) {
        _motorTypes = motorTypes;

        for(int i = 0; i<motorIDs.length; i++) {
            switch (motorTypes[i]) {
                case SPARK:
                    motorList.add(new Spark(motorIDs[i]));
                    break;
                case KRAKEN:
                default:
                    motorList.add(new Talon(motorIDs[i]));
                    break;
            }
        }

        for(PWMMotorController motor : motorList) {
            motor.setSafetyEnabled(true);
        }

        String[] defaultNames = new String[motorList.size()];
        for(int i = 0; i<defaultNames.length; i++) {
            defaultNames[i] = "Motor " + i;
        }

        setNameList(defaultNames);
    }

    public void invert(boolean inverted, int id) {
        if(id>-1 && id<motorList.size()) 
            motorList.get(id).setInverted(inverted);
    }

    public void invert(boolean inverted, String name) {
        if(nameList.contains(name))
            invert(inverted, nameList.indexOf(name));
    }

    public void makeFollowers(int followerID, int leaderID) {
        if(followerID>-1 && followerID<motorList.size() && leaderID>-1 && leaderID<motorList.size())
            motorList.get(leaderID).addFollower(motorList.get(followerID));
    }

    public void makeFollowers(String followerName, String leaderName) {
        if(nameList.contains(followerName)&&nameList.contains(leaderName))
            makeFollowers(nameList.indexOf(followerName), nameList.indexOf(leaderName));
    }

    public void setMotorVoltage(Voltage volts, int id) {
        if(id>-1 && id<motorList.size()) 
            motorList.get(id).setVoltage(volts);
    }

    public void setMotorVoltage(Voltage volts, String name) {
        if(nameList.contains(name)) 
            setMotorVoltage(volts, nameList.indexOf(name));
    }

    public void setMotorRPS(AngularVelocity angularVelocity, int id) {
        if(id>-1 && id<motorList.size()) {
            switch(_motorTypes[id]) {
                case KRAKEN:
                    motorList.get(id).set(angularVelocity.in(RevolutionsPerSecond)/100); //100 is for the free speed of the kraken, idk whats going on rn weird stuff, best current fix
                    break;
                case SPARK:
                default:
                    motorList.get(id).set(angularVelocity.in(RevolutionsPerSecond)/94.6); //same here but w/ neos vortexes are supposedly faster but without knowing which is on there i cant account for it (i.e. i dont want to fix the enum rn)
                    break;
            }
        }
    }

    public void setMotorRPS(AngularVelocity angularVelocity, String name) {
        if(nameList.contains(name))
            setMotorRPS(angularVelocity, nameList.indexOf(name));
    }

    public void setNameList(String[] names) {
        for(int i = 0; i<names.length; i++) {
            nameList.add(names[i]);
        }
    }

    public class FlexMotor extends PWMMotorController{

        private MotorType _motorType;

        public FlexMotor(String s, int i, MotorType motorType) {
            super(s, i);

            setMotorType(motorType);
        }

        public void setMotorType(MotorType motorType) {
            _motorType = motorType;
            switch(_motorType) {
                case SPARK:
                    m_pwm.setBoundsMicroseconds(2003, 1550, 1500, 1460, 999);
                    break;
                case KRAKEN:
                default:
                    m_pwm.setBoundsMicroseconds(2037, 1539, 1513, 1487, 989);
                    break;
            }
        }

        public MotorType getMotorType() {
            return _motorType;
        }

    }

    public enum MotorType {SPARK, KRAKEN}

}
