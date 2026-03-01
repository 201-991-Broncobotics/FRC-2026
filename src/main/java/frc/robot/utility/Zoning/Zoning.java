package frc.robot.utility.Zoning;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utility.Zoning.Zone;

import frc.robot.Constants.ZoneConstants;

public class Zoning {
    private boolean inZone = false; //In zone or out of zone

    private ArrayList<Zone> Zones = new ArrayList<Zone>();


    public Zoning(Zone zone){

        Zones.add(zone);
    }

    public Zoning(ArrayList<Zone> zones){
        Zones = zones;
    }

    public boolean inZones(Pose2d pose){
        for (Zone zone : Zones) {
            if(zone.inZone(pose))
                return true;
        }

        return false;
    }

    public boolean updateZones(Pose2d pose){
        inZone = inZones(pose);

        return inZone;
    }

    public boolean ifLeftZones(Pose2d Pose, Pose2d lastPose){
        for (Zone zone : Zones) {
            if(zone.ifLeftZone(lastPose, Pose)){
                return true;
            }
        }

        return false;
    }

    public boolean ifLeftZones(Pose2d Pose){
        if (!inZones(Pose) && inZone) {
            return true;
        } else {
            return false;
        }
    }

    public boolean ifEnteredZones(Pose2d Pose, Pose2d lastPose){
        for (Zone zone : Zones) {
            if(zone.ifEnteredZone(lastPose, Pose)){
                return true;
            }
        }

        return false;
    }

    public boolean ifEnteredZones(Pose2d Pose){
        if (inZones(Pose) && !inZone) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getZoningState(){
        return inZone;
    }


}
