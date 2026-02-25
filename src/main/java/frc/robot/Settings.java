package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AreaConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utility.Area;
import edu.wpi.first.math.util.Units;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */

public class Settings {

    public static boolean useNormalControls = false; // false is for single player/Aidan controls

    public static boolean tuningTelemetryEnabled = true;

    public static PPHolonomicDriveController PathFollowerController = new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(7.0, 0.0, 0.0) // Rotation PID constants
    );

    // Auto PathFinding constraints
    public static PathConstraints FollowerConstraints = new PathConstraints(
                    3.0, 4.0,
                    Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static class IntakeSettings {

        public static double runningPower = 0.5; 
        public static double defaultPower = 0; 

        public static double pivotMotorVelocity = 10.0; //7
        public static double pivotMotorAcceleration = 3.5; //2 

        public static double pivotkP = 6.0; 
        public static double pivotkI = 0; 
        public static double pivotkD = 0; 
        public static double pivotkG = 0.0; // 0.35
        
    }

    public static class TraverseSettings {

        public static double rollerMotorPower = 1.0; 
        public static double scoopMotorPower = 1.0;


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

        public static double kP = 1; // flywheels
        public static double kI = 0; 
        public static double kD = 0; 
        public static double kS = 0; 
        public static double kV = TurretConstants.maxForwardVoltage/TurretConstants.x44MaxRPM; 
        public static double kA = 0; 
        public static double tkP = 0; // turntable
        public static double tkI = 0; 
        public static double tkD = 0;  
        public static double hkP = 0; // hood
        public static double hkI = 0; 
        public static double hkD = 0; 
        public static double setVelocities = 1000; // rpm
        public static double targetTurntableAngle = 0; // degrees

        public static boolean autoLowerHood = true;

        public static ArrayList<Area> Areas = new ArrayList<>(Arrays.asList(AreaConstants.blueLeftTrench, AreaConstants.blueRightTrench, AreaConstants.redLeftTrench, AreaConstants.redRightTrench));

        public static boolean reverseCounterDirection = false;
    }


    public static class OuttakeTrajectorySettings {
        public static double HubXOffset = 10.4;
        public static double HubYOffset = 64;


        //Im sorry but maybe I'm going to far with this
        public static double Cd = 0.5; // drag coefficient - I might remove this
        public static double kL = 1.5; // lift constant
        public static double airDensity = 1.1; // kg/m^3

        public static double KD = airDensity * (Math.PI * Math.pow(5.91/2.0, 2)) * Cd / (0.203+0.227); // (0.203+0.227) is average mass / 2 * 2
        public static double KL = airDensity * (Math.PI * Math.pow(5.91/2.0, 2)) * kL / (0.203+0.227);
        public static double SpinTransferEfficiency = 0.3; // related to how much backspin the ball will get relative to roller speed


        // These change how many times the simulated trajectory is simulated
        public static double dt = 0.002; // time step for trajectory prediction
        public static double SolutionTolerance = 0.01; // how precise angle result is 



        public static double targetDistance = 100; // inches
        public static double targetHeight = 10; // inches
    }



    public static class AutoTargetingSettings {

        public static boolean AutoAimingEnabled = true;
        public static PIDController AutoTurningPID = new PIDController(0.9, 0, 0);
        public static PIDController AutoDrivingPID = new PIDController(0.9, 0, 0);
        public static double AutoDrivingMaxPower = 0.5;

        public static boolean AutoDrivingEnabled = true;
        public static double AutoDrivingPower = 0;
        public static double targetPercentageOfVisionBlocked = 0.2;

        public static double searchingSpeed = 0.5;

        public static double leftCorrectX = 0;
        public static double rightCorrectX = 0;

    }

}