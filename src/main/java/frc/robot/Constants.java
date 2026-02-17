// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utility.Vector2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
   public static class MotorConstants{
    
        public static final int intakeID = 0; 
        public static final int rollerPivotID = 2; 
        public static final int rollerPivot2ID = 3; 
        public static final int traverseMotor = 4; 
        public static final int leftFlyID = 5; 
        public static final int rightFlyID = 6;
        public static final int turntableID = 7;  
        public static final int turretPivotID = 8; 
        public static final int climberID = 9; 
        public static final int elevatorID = 10; 
      
    }
    public static class OperatorConstants {

        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1; 

    }

    public static class RollerConstants {

        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12; 

        public static final boolean currentLimitsEnabled = true;  
        public static final double supplyCurrent = 40; 
        public static final double statorCurrent = 60; 

        public static final double x60ShaftRadius = 4/25.4; //inches
        public static final double startingPosition = 0; //radians
        public static final double highLimitAngle = Math.toRadians(106); //radians
        public static final double gearRatio = (1/25); 

    }

    public static class StorageConstants {

        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12; 

        public static final boolean currentLimitsEnabled = true;  
        public static final double supplyCurrent = 40; 
        public static final double statorCurrent = 60; 
        
        public static final int vortexCurrentLimit = 80; //in amps 

        
    }

    public static class ClimbingConstants {

        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12;

        public static final boolean currentLimitsEnabled = true;  
        public static final double supplyCurrent = 60; 
        public static final double statorCurrent = 80; 

        public static final double x60ShaftRadius = 4/25.4; //inches
        public static final double gearRatio = 15; 
        public static final double ticksPerRev = 2048; 

    }

    public static class TurretConstants {

        public static final String limelightName = "limelight";
        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12; 
        public static final double x44MaxRPM = 7750; 

        public static final boolean currentLimitsEnabled = true; 
        public static final double supplyCurrent = 60; 
        public static final double statorCurrent = 80; 

        public static final double gravityInches = 386.0885826; // in inches per second

        public static final double maxHoodAngle = Math.toRadians(70.196461);
        public static final double minHoodAngle = Math.toRadians(70.196461 - 45);
        
        
    }

    public static class AutoDrivingConstants {
        public static double FieldLength = 17.5483;
        public static double FieldWidth = 8.0519;

        public static Vector2d RedReefCenter = new Vector2d(4.284788875 + FieldLength/2, -0.000099 + FieldWidth/2);
        public static Vector2d BlueReefCenter = new Vector2d(-4.284788875 + FieldLength/2, -0.000099 + FieldWidth/2);

    }
}
