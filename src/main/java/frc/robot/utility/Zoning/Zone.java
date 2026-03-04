package frc.robot.utility.Zoning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is to normalize driving and make it easier for Mael to not drive into people. 
 * It also determines which driver controller/joystick should take priority and allows them to be used interchangeably
 */
public class Zone {
    private Shape _shape;

    public Zone(Shape shape) {
        _shape = shape;
    }

    public boolean ifLeftZone(Pose2d lastPose, Pose2d Pose){
        if (lastPose == null) return false;
        
        // Return true if currently OUT of the shape, but previously IN the shape
        return !_shape.inArea(Pose.getTranslation()) && _shape.inArea(lastPose.getTranslation());
    }

    public boolean ifEnteredZone(Pose2d lastPose, Pose2d Pose){
        if (lastPose == null) return false;
        
        // Return true if currently IN the shape, but previously OUT of the shape
        return _shape.inArea(Pose.getTranslation()) && !_shape.inArea(lastPose.getTranslation());
    }

    public boolean inZone(Pose2d Pose){
        return _shape.inArea(Pose.getTranslation());
    }
}