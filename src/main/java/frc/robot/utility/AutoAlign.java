package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.ZoneConstants;
import frc.robot.utility.Zoning.Zoning;

public class AutoAlign {
    public static void AutoAlignDrive(Pose2d robotPose2d){
        if (Zoning.inZones(robotPose2d,ZoneConstants.TrenchZones)) {//In Trench Zones
            //rotate to 0
        } else if (Zoning.inZones(robotPose2d, ZoneConstants.ClimbZones)){
            if(robotPose2d.getY() > 4){
                // >Y side
            } else {
                // <Y Side
            }
        }
    }
}
