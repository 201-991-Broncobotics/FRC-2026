package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.math.controller.PIDController;

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

        public static double pivotkP = 5; 
        public static double pivotkI = 0; 
        public static double pivotkD = 0.2; 
        public static double pivotkG = 0.35; 
        
    }

    public static class StorageSettings {

        public static double runningTraverseMotor = 6; 


    }

    public static class TurretSettings {

        public static double runningLeftFlyVoltage = 10; 
        public static double runningRightFlyVoltage = 10; 
        
    }

    public static class ClimbingSettings{

        public static double runningClimbingVoltage = 12; 
        public static double runningElevatorVoltage = 12; 

        public static double climberMotorVelocity = 0.4; 
        public static double climberMotorAcceleration = 1.2; 
        public static double climberkP = 5; 
        public static double climberkI = 0; 
        public static double climberkD = 0.2; 
        public static double climberkG = 0.35; 

        public static double elevatorMotorVelocity = 0.4; 
        public static double elevatorMotorAcceleration = 1.2; 
        public static double elevatorkP = 5; 
        public static double elevatorkI = 0; 
        public static double elevatorkD = 0.2; 
        public static double elevatorkG = 0.35; 

    }
    public static class ExampleSettings {
        /*public static double kSE = 0.001;
        public static double kGE = 0.300;
        public static double kVE = 0.9;

        public static double kSA = 0.001;
        public static double kGA = 0.05; 
        public static double kVA = 0.05; 

        public static double elevatorTolerance =.8;
        public static double armTolerance = 5;
        public static double elevatorSpeedControl = 1;
        public static double elevatorRotationsToInches;

        public static double startingPosition = 0;
        public static double maxHeight = 50;
        public static double minHeight = 0;

        public static double manualControlSpeed = 25; // max speed in inches per second 


        public static double delayBeforeStaging = 750; // milliseconds that after holding the change stage button, will cause it to skip to max/min stage
        */ 
    }



}