package frc.robot.utility;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


/**
 * This is to normalize driving and make it easier for Mael to not drive into people. 
 * It also determines which driver controller/joystick should take priority and allows them to be used interchangeably
 */

public class Area {
    private enum Shape{
        RECTANGLE,
        CIRCLE
    };

    private Shape shape;

    private Pose2d p1, p2;

    public Area(Translation2d topLeftCorner, Translation2d bottomRightCorner) {
        p1 = new Pose2d(topLeftCorner, new Rotation2d(0));
        p1 = new Pose2d(bottomRightCorner, new Rotation2d(0));

        shape = Shape.RECTANGLE;
    }

    public Area(Pose2d polarCoords) {
        p1 = polarCoords;

        shape = Shape.RECTANGLE;
    }

    public boolean inArea(Pose2d pose){
        if(shape == Shape.RECTANGLE){
            return inAreaRectangle(pose);
        } else if(shape == Shape.RECTANGLE){
            return inAreaCircle(pose);
        } else {
            return false;
        }
    }

    private boolean inAreaRectangle(Pose2d pose){
        if((pose.minus(p1).getX() > 0 && pose.minus(p1).getY() < 0) && (pose.minus(p2).getX() < 0 && pose.minus(p2).getY() > 0)){
            return true;
        } else {
            return false;
        }
    }

    private boolean inAreaCircle(Pose2d pose){
        if((pose.minus(p1).getX() > 0 && pose.minus(p1).getY() < 0) && (pose.minus(p2).getX() < 0 && pose.minus(p2).getY() > 0)){
            return true;
        } else {
            return false;
        }
    }
}
