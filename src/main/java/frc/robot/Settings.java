package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.TurretConstants;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */

public class Settings {

    public static boolean useNormalControls = false; // false is for single player/Aidan controls

    public static class RollerSettings {

        public static double runningVoltage = 6; 
        public static double defaultVoltage = 0; 

        public static double pivotMotorVelocity = 0.4; 
        public static double pivotMotorAcceleration = 1.2; 

        public static double pivotkP = 1; 
        public static double pivotkI = 0; 
        public static double pivotkD = 0.2; 
        public static double pivotkG = 0.35; 
        
    }

    public static class StorageSettings {

        public static double runningTraverseMotor = 6; 


    }

    public static class ClimbingSettings{

        public static double startingDistance = 0; 

        public static double climberDistance = 13; //inches
        public static double elevatorDistance = 8; //inches

        public static double runningClimbingVoltage = 12; 
        public static double runningElevatorVoltage = 12; 

        public static double climberMotorVelocity = 0.4; 
        public static double climberMotorAcceleration = 1.2; 
        public static double climberkP = 1; 
        public static double climberkI = 0; 
        public static double climberkD = 0.2; 
        public static double climberkG = 0.35; 

        public static double elevatorMotorVelocity = 0.4; 
        public static double elevatorMotorAcceleration = 1.2; 
        public static double elevatorkP = 1; 
        public static double elevatorkI = 0; 
        public static double elevatorkD = 0.2; 
        public static double elevatorkG = 0.35; 

    }

    public static class TurretSettings {

        public static double kP = 1; 
        public static double kI = 0; 
        public static double kD = 0.2; 
        public static double kS = 0; 
        public static double kV = TurretConstants.maxForwardVoltage/TurretConstants.x44MaxRPM; 
        public static double kA = 0; 
        public static double setVelocities = 0; 
        
    }



}