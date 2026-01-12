package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.math.controller.PIDController;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */

public class Settings {

    public static boolean useNormalControls = false; // false is for single player/Aidan controls
    public static boolean tuningTelemetryEnabled = true;

    public static class RollerSettings {

        public static double startingVoltage = 6; 
        
    }
    public static class SomethingSettings {
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

    public static class OtherSettings {
        //public static double startRoll = Math.toRadians(0);
        //public static double startPitch = Math.toRadians(0);

        //public static PIDController LeftDiffyPID = new PIDController(0.09, 0, 0); 
        //public static PIDController RightDiffyPID = new PIDController(0.09, 0, 0); 

    }

    public static class AutoTargetingSettings {

        public static boolean AutoAimingEnabled = true;
        public static PIDController AutoAimPID = new PIDController(0, 0, 0);

        public static boolean AutoDrivingEnabled = false;
        public static double AutoDrivingPower = 0;
        public static double targetPercentageOfVisionBlocked = 0.2;

        public static double searchingSpeed = 0.5;

        public static double leftReefCrosshairOffset = 0;

    }
}