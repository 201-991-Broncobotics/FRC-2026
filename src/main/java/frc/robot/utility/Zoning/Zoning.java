package frc.robot.utility.Zoning;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ZoneConstants;
import frc.robot.utility.Zoning.Zone;

public class Zoning {
    private boolean inZone = false; // Currently in zone
    private boolean prevInZone = false; // Was in zone previously

    private ArrayList<Zone> Zones = new ArrayList<Zone>();

    public Zoning(Zone zone){
        this.Zones.add(zone);
    }

    public Zoning(ArrayList<Zone> zones){
        this.Zones = zones;
    }

    // Purely checks if a pose is in the zones, without modifying the state.
    public boolean inZones(Pose2d pose){
        for (Zone zone : Zones) {
            if(zone.inZone(pose)) {
                return true;
            }
        }
        return false;
    }

    // Updates the internal tracking state. Call this once per robot loop.
    public boolean updateZones(Pose2d pose){
        prevInZone = inZone;
        inZone = inZones(pose);
        return inZone;
    }

    // Checks if the robot moved from lastPose (inside) to Pose (outside)
    public boolean ifLeftZones(Pose2d Pose, Pose2d lastPose){
        for (Zone zone : Zones) {
            if(zone.ifLeftZone(lastPose, Pose)){
                return true;
            }
        }
        return false;
    }

    // Checks if the robot just left the zone compared to the last updated state
    public boolean ifLeftZones(Pose2d Pose){
        // The pose is OUTSIDE, but the last recorded state was INSIDE
        if (!inZones(Pose) && inZone) {
            return true;
        } else {
            return false;
        }
    }

    // Checks if the robot moved from lastPose (outside) to Pose (inside)
    public boolean ifEnteredZones(Pose2d Pose, Pose2d lastPose){
        for (Zone zone : Zones) {
            if(zone.ifEnteredZone(lastPose, Pose)){
                return true;
            }
        }
        return false;
    }

    // Checks if the robot just entered the zone compared to the last updated state
    public boolean ifEnteredZones(Pose2d Pose){
        // The pose is INSIDE, but the last recorded state was OUTSIDE
        if (inZones(Pose) && !inZone) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getZoningState(){
        return inZone;
    }

    public void setZone(Zone zone){
        this.Zones.clear();
        this.Zones.add(zone);
    }

    public void setZones(ArrayList<Zone> zones){
        this.Zones.clear();
        this.Zones = zones;
    }

    public void addZone(Zone zone){
        this.Zones.add(zone);
    }

    public Pose2d getCenterPose2d(){
        return Zones.get(0).getCenterPose2d();
    }

    public static boolean inZones(Pose2d pose, ArrayList<Zone> zones){
        for (Zone zone : zones) {
            if(zone.inZone(pose)) {
                return true;
            }
        }
        return false;
    }
}