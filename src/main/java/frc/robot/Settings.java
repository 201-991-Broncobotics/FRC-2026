package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */

public class Settings {

    public static boolean tuningTelemetryEnabled = true;

    public static PIDConstants translationPIDConstants = new PIDConstants(0.5, 0.0, 0.0);
    public static PIDConstants rotationPIDConstants = new PIDConstants(1.0, 0.0, 0.0);

    public static boolean useRLimelight = true;
    public static boolean useLLimelight = true;

    public static PPHolonomicDriveController PathFollowerController = new PPHolonomicDriveController( // PPHolonomicController is the built-in path following controller for holonomic drive trains
        translationPIDConstants, // Translation PID constants
        rotationPIDConstants // Rotation PID constants
    );

    // Auto PathFinding constraints
    public static PathConstraints FollowerConstraints = new PathConstraints(
                    3.0, 4.0,
                    Units.degreesToRadians(540), Units.degreesToRadians(720));


    public static double safetyDistanceFromWall = (2.5) / 39.37; // the closest distance the robot will let the drive go to the wall in meters (inches)
    public static boolean keepWithinPerimeter = false;
    public static double TranslationKP = 0.5;

    public static class IntakeSettings {

        public static double runningPower = 1; 
        public static double reversePower = -0.75; 

        public static double pivotMotorVelocity = 5; //7
        public static double pivotMotorAcceleration = 5; //2 

        public static double pivotkP = 2.0; 
        public static double pivotkI = 0; 
        public static double pivotkD = 0; 
        public static double pivotkG = 0.0; // 0.35

        public static boolean autoControl = false;

        public static final double STALL_CURRENT_THRESHOLD = 20.0; // Amps
        public static final double STALL_VELOCITY_THRESHOLD = 0.1; // Rotations per second
        
        public static double airShooterPivotAngle = Math.toRadians(20); // this is set to be correct where 90 is up and 0 is straight out

        public static double agitatePulsePeriod = 0.6; // seconds

        public static double customKP = 0.2;
        public static double customKG = -0.07;
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

        public static double startingPosition = 0; 

        public static double climberMotorVelocity = 0.4; 
        public static double climberMotorAcceleration = 1.2; 
        public static double climberkP = 1; 
        public static double climberkI = 0; 
        public static double climberkD = 0.001; 
        public static double climberkG = 0.35; 

        public static double elevatorMotorVelocity = 100; 
        public static double elevatorMotorAcceleration = 20; 
        public static double elevatorkP = 4; 
        public static double elevatorkI = 0; 
        public static double elevatorkD = 0.001; 
        public static double elevatorkG = 0.35; 

    }

    public static class TurretSettings {

        public static double kP = 0.3; // flywheels
        public static double kI = 0; 
        public static double kD = 0; 
        public static double kS = 0; 
        public static double kV = 0.12; 
        public static double kA = 0; 
        public static double tkP = 25; // turntable
        public static double tkI = 0; 
        public static double tkD = 0;  
        public static double tkS = 2.0;
        public static double tkV = 1;
        public static double hkP = 1; // hood
        public static double hkI = 0; 
        public static double hkD = 0; 
        public static double setVelocities = 2000; // rpm

        public static boolean autoLowerHood = true;

        public static boolean reverseCounterDirection = false;

        public static double hoodCalibrationDownTime = 0.1; // time spent going to down position (which should be short since it already is in the down position)
        public static double hoodCalibrationUpTime = 0.25; // time spent going up after it has reached its underestimate for max height
        public static double hoodCalibrationPower = 0.4;

        public static double defaultTurretPosition = Math.toRadians(170); // position at slight angle to allow intake to go up
        public static double minTurretAngle = Math.toRadians(-45);
        public static double maxTurretAngle = Math.toRadians(180);

        public static double TurretAbsoluteOffset = Math.toRadians(130.9);

        public static int numberOfIterations = 5; // 1 frame = about 20ms
        public static double TurntableDeadband = Math.toRadians(1.5);
        public static double FlywheelDeadband = 30; // rpm
        public static double HoodDeadband = Math.toRadians(0.5);

        public static boolean tuningMode = false;
    }


    public static class Traj { // prob don't need another class for outtake but like, it has a shorter name
        public static final double a1 = 0.0943943432755;
        public static final double a2 = 0.00000113951263818;

        public static final double b1 = -28.695386025;
        public static final double b2 = 2.25026043353;
        public static final double b3 = -0.0117303333665;

        public static final double k1 = -0.00276176; // n1
        public static final double k2 = -4082.94445; // n2
        public static final double k3 = 22.56115; // n3

        // temporary or maybe permanent simplified regression version
        public static final double m1 = -409.73231;
        public static final double m2 = 129.5842;
        public static final double m3 = -0.766076;
        public static final double m4 = 0.585504;
        public static final double m5 = -380.63038;

        public static final double g1 = 0.000312494918492; // actual k1
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

    public static class RobotSettings{
        public static final boolean overrideMode = true;
    }

}