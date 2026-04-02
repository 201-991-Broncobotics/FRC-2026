// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utility.Vector2d;
import frc.robot.utility.Zoning.Shape;
import frc.robot.utility.Zoning.Zone;
import frc.robot.utility.Zoning.Zoning;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double RobotWidth = 0.947917; // including bumpers and at max extension of storage in meters
    public static double RobotLength = 0.824083 + 0.203250;
    public static double ForwardCenterDist = 0.203250/2.0;
  
    public static class MotorConstants{
    
        public static final int intakeID = 14; 
        public static final int traverseRollerID = 16; 
        public static final int traverseRoller2ID = 17; 
        public static final int rightIntakePivotID = 18; 
        public static final int traverseScoopID = 15; 
        public static final int leftFlyID = 11; 
        public static final int rightFlyID = 12;
        public static final int turntableID = 13;  
        public static final int hoodMotorID = 10; 
        public static final int elevatorID = 9; 
      
    }
    public static class OperatorConstants {

        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1; 

    }

    public static class IntakeConstants {

        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12; 

        public static final boolean currentLimitsEnabled = true;  
        public static final double supplyCurrent = 15; // helps prevent brownouts
        public static final double statorCurrent = 40; // helps prevent motor overheating
        public static final double pivotSupplyCurrent = 20; // 20
        public static final double pivotStatorCurrent = 30; // 30

        public static final double x60ShaftRadius = 4/25.4; //inches
        public static final double outIntakePosition = Math.toRadians(0); //radians, also I hate this cause I can't switch these without the motion profile breaking
        public static final double upIntakePosition = Math.toRadians(95); //radians
        //public static double maxPivotAngle = lowLimitAngle; //radians
        //public static double minPivotAngle = highLimitAngle; //radians
        //public static final double startingPosition = highLimitAngle; //radians
        public static final double gearRatio = (1.0/25.0); 

    }

    public static class TraverseConstants {

        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12; 

        public static final boolean currentLimitsEnabled = true;  
        public static final double rollerSupplyCurrent = 25; 
        public static final double rollerStatorCurrent = 40; // disabled
        public static final double scoopSupplyCurrent = 25; 
        public static final double scoopStatorCurrent = 40; // disabled
    }

    public static class ClimbingConstants {

        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12;

        public static final boolean currentLimitsEnabled = true;  
        public static final double supplyCurrent = 40; 
        public static final double statorCurrent = 60; // disabled

        public static final double x60ShaftRadius = 4/25.4; //inches
        public static final double gearRatio = (1.0/15.0); 
        public static final double ticksPerRev = 2048; 

        public static final double maxLimitPosition = Math.toRadians(10848.515625); 

        public static final double elavatorActualPositionOffset = 46.760744;
    }

    public static class TurretConstants {

        public static final String limelightName = "limelight";
        public static final double maxForwardVoltage = 12; 
        public static final double maxReverseVoltage = -12; 
        public static final double x44MaxRPM = 7750; 

        public static final boolean currentLimitsEnabled = true; 
        public static final double supplyCurrent = 50; 
        public static final double statorCurrent = 80; // disabled
        public static final int hoodMotorCurrent = 30;
        public static final double turretSupplyCurrent = 40;
        public static final double turretStatorCurrent = 60;

        public static final double gravityInches = 386.0885826; // in inches per second

        public static final double maxHoodAngle = Math.toRadians(70.196461);
        public static final double minHoodAngle = Math.toRadians(70.196461 - 45);     

        // maxHoodMotorRot and minHoodMotorRot are now updated actively in the outtakeSubsystem
        
        public static final double counterThreshold = 0; //Degrees

        public static final double incrementAngle = 1.5; //Degrees
    }

    public static class ZoneConstants {//Zones on the field from the POV of blue Human player
        public static double FieldLength = 16.57; // these are the coords in Choreo I have no idea why are different then the pdf pages but I trust choreo more
        public static double FieldWidth = 8.1;

        public static Zone blueLeftTrench = new Zone(new Shape.Rectangle(new Translation2d(4.6,7.25), 2, 1.6));
        public static Zone blueRightTrench = new Zone(new Shape.Rectangle(new Translation2d(4.6,0.8), 2, 1.6));
        public static Zone redLeftTrench = new Zone(new Shape.Rectangle(new Translation2d(11.9,7.25), 2, 1.6));
        public static Zone redRightTrench = new Zone(new Shape.Rectangle(new Translation2d(11.9,0.8), 2, 1.6));

        public static ArrayList<Zone> TrenchZones = new ArrayList<>(Arrays.asList(ZoneConstants.blueLeftTrench, ZoneConstants.blueRightTrench, ZoneConstants.redLeftTrench, ZoneConstants.redRightTrench));

        public static Zone blueLeftRamp = new Zone(new Shape.Rectangle(new Translation2d(4.6,5.5), 2,2.25));
        public static Zone blueRightRamp = new Zone(new Shape.Rectangle(new Translation2d(4.6,2.5), 2, 2.25));
        public static Zone redLeftRamp = new Zone(new Shape.Rectangle(new Translation2d(11.9,5.5), 2, 2.25));
        public static Zone redRightRamp = new Zone(new Shape.Rectangle(new Translation2d(11.9,2.5), 2, 2.25));

        public static ArrayList<Zone> RampZones = new ArrayList<>(Arrays.asList(ZoneConstants.blueLeftTrench, ZoneConstants.blueRightTrench, ZoneConstants.redLeftTrench, ZoneConstants.redRightTrench));


        public static Zone blueClimb = new Zone(new Shape.Rectangle(new Translation2d(0,3.75), new Translation2d(1.5,3.5)));
        public static Zone redClimb = new Zone(new Shape.Rectangle(new Translation2d(15,4.25), new Translation2d(FieldLength,3.5)));

        public static ArrayList<Zone> ClimbZones = new ArrayList<>(Arrays.asList(ZoneConstants.blueLeftTrench, ZoneConstants.blueRightTrench, ZoneConstants.redLeftTrench, ZoneConstants.redRightTrench));

        public static Zone ballsZone = new Zone(new Shape.Rectangle(new Translation2d(FieldLength/2.0,FieldWidth/2.0), 2,5));

        public static Zone middleZone = new Zone(new Shape.Rectangle(new Translation2d(4.75,FieldWidth), new Translation2d(12,0)));
        public static Zone blueZone = new Zone(new Shape.Rectangle(new Translation2d(0,FieldWidth), new Translation2d(4.5,0)));
        public static Zone redZone = new Zone(new Shape.Rectangle(new Translation2d(11.75,FieldWidth), new Translation2d(FieldLength,0)));

        public static Translation3d blueHub = new Translation3d(4.6,FieldWidth/2.0, 1.83); // new Translation3d(4.6,4, 1.83);
        public static Translation3d redHub = new Translation3d(11.9,FieldWidth/2.0, 1.83); // new Translation3d(11.9,4, 1.83);
        public static double hubWidth = 2;

        public static boolean alliance = true; //True is blue
        public static Translation3d allianceHub = blueHub;
        public static Zoning allianceZone = new Zoning(blueZone);

        public static Zone fieldZone = new Zone(new Shape.Rectangle(new Translation2d(0, FieldWidth), new Translation2d(FieldLength, 0)));

        public static Zoning CCWTurnAreas = new Zoning(
            new Zone(new Shape.Rectangle(new Translation2d(0, 4), new Translation2d(4.6, 0))),
            new Zone(new Shape.Rectangle(new Translation2d(4.6, FieldWidth), new Translation2d(FieldLength/2.0, FieldWidth/2.0))),
            new Zone(new Shape.Rectangle(new Translation2d(FieldLength/2.0, FieldWidth/2.0), new Translation2d(11.9, 0))),
            new Zone(new Shape.Rectangle(new Translation2d(11.9, FieldWidth), new Translation2d(FieldLength, FieldWidth/2.0)))
        );
    }

    
}
