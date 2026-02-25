package frc.robot.utility;

import java.io.ObjectOutputStream.PutField;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.utility.Shape;


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
        if(!_shape.inArea(Pose.getTranslation()) && _shape.inArea(lastPose.getTranslation())){
            return true;
        } else {
            return false;
        }
    }

    public boolean ifEnteredZone(Pose2d lastPose, Pose2d Pose){
        if(!_shape.inArea(Pose.getTranslation()) && _shape.inArea(lastPose.getTranslation())){
            return true;
        } else {
            return false;
        }
    }
}
