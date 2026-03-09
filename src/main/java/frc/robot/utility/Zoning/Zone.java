package frc.robot.utility.Zoning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


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

    public Pose2d getCenterPose2d(){
        return new Pose2d(_shape.getCenterPoint(), new Rotation2d(0));
    }

    public Shape getShape() { return _shape; }
}