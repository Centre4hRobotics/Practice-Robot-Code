package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.ArrayList;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlexKrakMech extends SubsystemBase{
    
    private ArrayList<Talon> motorList = new ArrayList<Talon>();

    public FlexKrakMech(int[] motorIDs) {
        for(int id : motorIDs) {
            motorList.add(new Talon(id));
        }
        for(Talon motor : motorList) {
            motor.setSafetyEnabled(true);
        }
    }

    public void invert(boolean inverted, int id) {
        motorList.get(id).setInverted(inverted);
    }

    public void makeFollowers(int followerID, int leaderID) {
        motorList.get(leaderID).addFollower(motorList.get(followerID));
    }

    public void setMotorVoltage(Voltage volts, int id) {
        if(id>-1 && id<motorList.size()) {
            motorList.get(id).setVoltage(volts);
        }
    }

    public void setMotorRPS(AngularVelocity angularVelocity, int id) {
        if(id>-1 && id<motorList.size()) {
            motorList.get(id).set(angularVelocity.in(RevolutionsPerSecond)/600); //600 is for the free speed of the kraken, idk whats going on rn weird stuff, best current fix
        }
    }

}
