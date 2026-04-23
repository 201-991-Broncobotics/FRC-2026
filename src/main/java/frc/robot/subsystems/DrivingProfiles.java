package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Constants.ZoneConstants;
import frc.robot.Settings.AutoTargetingSettings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain.gyroData;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import frc.robot.utility.Zoning.Shape;
import frc.robot.utility.Zoning.Zone;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.Vector2d;

/**
 * This is to normalize driving and make it easier for Mael to not drive into people. 
 * It also determines which driver controller/joystick should take priority and allows them to be used interchangeably
 */
public class DrivingProfiles extends SubsystemBase {

    private DoubleSupplier fowardControllerInput, strafeControllerInput, rotationControllerInput, throttleControllerInput, rotationThrottleControllerInput;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private double controllerDriveCurveMag, controllerTurnCurveMag;

    private final double AutoThrottleDeadband = 0.05;

    private boolean autoDriving = false;

    private double autoForwardOutput = 0, autoStrafeOutput = 0, autoRotationOutput = 0;

    private static CommandSwerveDrivetrain drivetrain;

    //private boolean useAutoDrivingThrottle = true;
    //private DoubleSupplier AutoDrivingThrottle;

    private static Pose2d RobotPose;

    private ElapsedTime FPSTimer;

    public List<List<Pose2d>> FieldTargetPoints; // Blue then red

    private Pose2d ClosestFieldTargetPoint = new Pose2d();

    public static boolean allowedToUseLimelight = true;
    public boolean robotCentricOverride = false;

    private double BatteryVoltage = 14;
    public static int currentDriveSupplyCurrentLimit = 120;
    private ElapsedTime CurrentLimitTimer;

    private static CommandXboxController controller1, controller2;
    private static boolean haveControllerObjects = false;
    private int rumbleIndex = 0; 
    private double rumbleEndTime = 0;
    private ElapsedTime rumbleTimer;

    public static boolean runRumble = false, autoWasJustRan = false;
    private boolean autoAlign = false;


    public DrivingProfiles(CommandSwerveDrivetrain drivetrain) {
        DrivingProfiles.drivetrain = drivetrain;

        allowedToUseLimelight = true;

        RobotPose = drivetrain.getState().Pose;

        FPSTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        CurrentLimitTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        rumbleTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        if (Settings.tuningTelemetryEnabled) {
            /* 
            SmartDashboard.putNumber("Tune Auto Turning kP", AutoTargetingSettings.AutoTurningPID.getP());
            SmartDashboard.putNumber("Tune Auto Turning kI", AutoTargetingSettings.AutoTurningPID.getI());
            SmartDashboard.putNumber("Tune Auto Turning kD", AutoTargetingSettings.AutoTurningPID.getD());
            SmartDashboard.putNumber("Tune Auto Driving kP", AutoTargetingSettings.AutoDrivingPID.getP());
            SmartDashboard.putNumber("Tune Auto Driving kD", AutoTargetingSettings.AutoDrivingPID.getD());
            SmartDashboard.putNumber("Tune Auto Driving Max Power", AutoTargetingSettings.AutoDrivingMaxPower);

            SmartDashboard.putBoolean("Auto Aiming Enabled", AutoTargetingSettings.AutoAimingEnabled);
            SmartDashboard.putBoolean("Auto Driving Enabled", AutoTargetingSettings.AutoDrivingEnabled);
            SmartDashboard.putNumber("Auto Driving Power", AutoTargetingSettings.AutoDrivingPower);
            SmartDashboard.putNumber("Auto target percentage of blocked vision", AutoTargetingSettings.targetPercentageOfVisionBlocked); */
        }
        SmartDashboard.putBoolean("Keep Robot Within Perimeter", Settings.keepWithinPerimeter);

        LimelightHelpers.setPipelineIndex("limelight", 0);

    }


    public void getControllers(CommandXboxController Controller1, CommandXboxController Controller2) {
        controller1 = Controller1;
        controller2 = Controller2;
        haveControllerObjects = true;
    }


    public void setUpControllerInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, DoubleSupplier rotationThrottleInput, double driveCurveMag, double turnCurveMag) {
        this.fowardControllerInput = fowardInput;
        this.strafeControllerInput = strafeInput;
        this.rotationControllerInput = rotationInput;
        this.throttleControllerInput = throttleInput;
        this.rotationThrottleControllerInput = rotationThrottleInput;
        this.controllerDriveCurveMag = driveCurveMag;
        this.controllerTurnCurveMag = turnCurveMag;
    }


    public void update() {
        if (updateController());
        else stopDriving();

        //if (useAutoDrivingThrottle) autoDriving = (AutoDrivingThrottle.getAsDouble() > AutoThrottleDeadband);
        if (autoDriving) updateAutoDriving();

        keepRobotInPerimeter();

        updateAutoAlign();

    }

// 6/7/1956 - TODO: Fix. This is temporary. - John Maxwell
// 2/16/2026 - This is was NOT temporary.

    private boolean updateController() {
        double forward = fowardControllerInput.getAsDouble();
        double strafe = strafeControllerInput.getAsDouble();
        double turn = rotationControllerInput.getAsDouble();
        double throttle = throttleControllerInput.getAsDouble();
        double rotationThrottle = rotationThrottleControllerInput.getAsDouble();

        double Direction = Math.atan2(forward, strafe);
        if (robotCentricOverride) Direction += RobotPose.getRotation().getRadians() + Math.toRadians(180);
        double joystickPower = Math.hypot(forward, strafe);
        double drivePower = Functions.throttleCurve(joystickPower, controllerDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, controllerTurnCurveMag) * rotationThrottle;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }


    public void updateAutoAlign() {

        double angleDifference;
        if (ZoneConstants.CCWTurnAreas.inZones(RobotPose)) angleDifference = Functions.angleDifference(RobotPose.getRotation().getRadians() + (Math.toRadians(45) - Settings.switchTurningDirection), 0, Math.toRadians(90)) + (Math.toRadians(45) - Settings.switchTurningDirection);
        else angleDifference = Functions.angleDifference(RobotPose.getRotation().getRadians() - (Math.toRadians(45) - Settings.switchTurningDirection), 0, Math.toRadians(90)) - (Math.toRadians(45) - Settings.switchTurningDirection);

        SmartDashboard.putNumber("Auto Align Power", angleDifference * 1.0);
        if (autoAlign) rotationOutput += angleDifference * 1.0;

    }


    public void updateAutoDriving() {

        RobotPose = drivetrain.getState().Pose;
        if (RobotPose != null) {
            Vector2d autoDrivingDirection = new Vector2d()
                .withMag(Functions.minMaxValue(0, AutoTargetingSettings.AutoDrivingMaxPower, 
                    AutoTargetingSettings.AutoDrivingPID.calculate(0, 
                        (new Vector2d(RobotPose.getX(), RobotPose.getY())).distFrom(new Vector2d(ClosestFieldTargetPoint.getX(), ClosestFieldTargetPoint.getY()))
                    )))
                .withAngle(((new Vector2d(RobotPose.getX(), RobotPose.getY())).minus(new Vector2d(ClosestFieldTargetPoint.getX(), ClosestFieldTargetPoint.getY()))).angle() + Math.toRadians(90));

            double throttle = 0; /* 
            if (preferController) {
                if (autoThrottleControllerInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleControllerInput.getAsDouble();
                else throttle = autoThrottleJoystickInput.getAsDouble();
            } else {
                if (autoThrottleJoystickInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleJoystickInput.getAsDouble();
                else throttle = autoThrottleControllerInput.getAsDouble();
            } */

            //throttle = Functions.deadbandValue(AutoDrivingThrottle.getAsDouble(), AutoThrottleDeadband);

            autoForwardOutput = autoDrivingDirection.y * throttle;
            autoStrafeOutput = autoDrivingDirection.x * throttle;
            autoRotationOutput = AutoTargetingSettings.AutoTurningPID.calculate(Functions.normalizeAngle(RobotPose.getRotation().getRadians() - ClosestFieldTargetPoint.getRotation().getRadians()), 0) * throttle;

            if (AutoTargetingSettings.AutoDrivingEnabled) {
                forwardOutput += autoForwardOutput;
                strafeOutput += autoStrafeOutput;
                rotationOutput += autoRotationOutput;
            }
            SmartDashboard.putString("AutoDriving", "is supposed to be working");
        } else SmartDashboard.putString("AutoDriving", "error");

        
    } 

    public double getAutoDrivePower() {
        return Math.hypot(autoForwardOutput, autoStrafeOutput);
    }

    public double getAutoDriveTurnPower() {
        return autoRotationOutput;
    }

    public void keepRobotInPerimeter() {
        Pose2d CenterOfRobotMech = RobotPose; //.plus(new Transform2d(Constants.ForwardCenterDist * Math.cos(RobotPose.getRotation().getRadians()), Constants.ForwardCenterDist * Math.sin(RobotPose.getRotation().getRadians()), new Rotation2d(0)));

        double effectiveWidth = 2 * Math.max(
            Math.hypot(Constants.RobotWidth/2.0, Constants.RobotLength/2.0) * Math.cos(CenterOfRobotMech.getRotation().getRadians() + Math.atan2(Constants.RobotLength, Constants.RobotWidth)),
            Math.hypot(Constants.RobotWidth/2.0, Constants.RobotLength/2.0) * Math.cos(CenterOfRobotMech.getRotation().getRadians() - Math.atan2(Constants.RobotLength, Constants.RobotWidth)));
        double effectiveLength = 2 * Math.max(
            Math.hypot(Constants.RobotWidth, Constants.RobotLength) * Math.sin(CenterOfRobotMech.getRotation().getRadians() + Math.atan2(Constants.RobotLength, Constants.RobotWidth)),
            Math.hypot(Constants.RobotWidth, Constants.RobotLength) * Math.sin(CenterOfRobotMech.getRotation().getRadians() - Math.atan2(Constants.RobotLength, Constants.RobotWidth)));

        Zone effectiveZone = new Zone(new Shape.Rectangle(
            ZoneConstants.fieldZone.getCenterPose2d().getTranslation(), // center
            ZoneConstants.fieldZone.getShape().getWidth() - 2*(effectiveLength/2.0 + Settings.safetyDistanceFromWall), // width
            ZoneConstants.fieldZone.getShape().getHeight() - 2*(effectiveWidth/2.0 + Settings.safetyDistanceFromWall) // height
            )); 

        double top = (effectiveZone.getCenterPose2d().getY() + effectiveZone.getShape().getHeight()/2.0) - CenterOfRobotMech.getY();
        double bottom = (effectiveZone.getCenterPose2d().getY() - effectiveZone.getShape().getHeight()/2.0) - CenterOfRobotMech.getY();
        double right = (effectiveZone.getCenterPose2d().getX() + effectiveZone.getShape().getWidth()/2.0) - CenterOfRobotMech.getX();
        double left = (effectiveZone.getCenterPose2d().getX() - effectiveZone.getShape().getWidth()/2.0) - CenterOfRobotMech.getX();

        boolean withinWidth = (Math.abs(CenterOfRobotMech.getX() - effectiveZone.getCenterPose2d().getX())) <= effectiveZone.getShape().getWidth()/2.0;
        boolean withinLength = (Math.abs(CenterOfRobotMech.getY() - effectiveZone.getCenterPose2d().getY())) <= effectiveZone.getShape().getHeight()/2.0;

        if (Settings.keepWithinPerimeter) {

            if (withinWidth) forwardOutput = (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * Functions.minMaxValue(Settings.TranslationKP * left, Settings.TranslationKP * right, (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * forwardOutput);
            else forwardOutput = (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * Settings.TranslationKP * Functions.closestToZero(right, left);

            if (withinLength) strafeOutput = Functions.minMaxValue(Settings.TranslationKP * bottom, Settings.TranslationKP * top, strafeOutput);
            else strafeOutput = Settings.TranslationKP * Functions.closestToZero(top, bottom);

        } else {
            if (withinWidth) SmartDashboard.putNumber("Driving Perms forward", (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * Functions.minMaxValue(Settings.TranslationKP * left, Settings.TranslationKP * right, (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * forwardOutput));
            else SmartDashboard.putNumber("Driving Perms forward", (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * Settings.TranslationKP * effectiveZone.getShape().getDistanceFromX(CenterOfRobotMech.getTranslation()));
            
            if (withinLength) SmartDashboard.putNumber("Driving Perms strafe", Functions.minMaxValue(Settings.TranslationKP * bottom, Settings.TranslationKP * top, strafeOutput));
            else SmartDashboard.putNumber("Driving Perms strafe", Settings.TranslationKP * effectiveZone.getShape().getDistanceFromY(CenterOfRobotMech.getTranslation()));
            

            SmartDashboard.putNumber("Driving Perms forward output", forwardOutput);
            SmartDashboard.putNumber("Driving Perms strafe output", strafeOutput);
            SmartDashboard.putBoolean("Driving Perms within length", withinLength);
            SmartDashboard.putBoolean("Driving Perms within the width", withinWidth);
        }
        
        
    }

    public void stopDriving() {
        forwardOutput = 0;
        strafeOutput = 0;
        rotationOutput = 0;
    }

    public void stopUsingCamera() { allowedToUseLimelight = false; }
    public void startUsingCamera() { allowedToUseLimelight = true; }

    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }

    public void enableAutoDriving() { autoDriving = true; }
    public void disableAutoDriving() { autoDriving = false; }

    public void enableAutoAlign() { autoAlign = true; }
    public void disableAutoAlign() { autoAlign = false; }

    public void enableRobotCentric() { robotCentricOverride = true; }
    public void disableRobotCentric() { robotCentricOverride = false; }

    public void autoWasJustRun() { autoWasJustRan = true; }
    public void teleOpWasJustRun() { 
        if (autoWasJustRan) runRumble = true;
        autoWasJustRan = false; 
    }

    public void setupAutoDrivingThrottle(DoubleSupplier AutoThrottle) {
        //useAutoDrivingThrottle = true;
        //AutoDrivingThrottle = AutoThrottle;
    }

    public static void DisableRightLimelight() {
        Settings.useRLimelight = false;
    }
    public static void DisableLeftLimelight() {
        Settings.useLLimelight = false;
    }
    public static void DisableLimelights() {
        Settings.useRLimelight = false;
        Settings.useLLimelight = false;
    }


    @Override
    public void periodic() {
        if (FPSTimer != null) {
            if (FPSTimer.time() > 0) SmartDashboard.putNumber("FPS:", Functions.round(1.0 / FPSTimer.time(), 2));
            FPSTimer.reset();
        }


        // FEEL THE RUMBLE
        if (rumbleIndex >= Settings.rumbleTimes.length) {
            runRumble = false;
            rumbleIndex = 0;
        }

        /* 
        if (haveControllerObjects && runRumble) {
            
            if (Timer.getMatchTime() == 0) {
                controller1.setRumble(RumbleType.kBothRumble, 0);
                controller2.setRumble(RumbleType.kBothRumble, 0);
            } else if (Settings.rumbleTimes[rumbleIndex][0] > Timer.getMatchTime()) {
                rumbleEndTime = Settings.rumbleTimes[rumbleIndex][1];
                rumbleTimer.reset();
                rumbleIndex++;
            }

            if (rumbleTimer.time() < rumbleEndTime) {
                controller1.setRumble(RumbleType.kBothRumble, 0.5);
                controller2.setRumble(RumbleType.kBothRumble, 0.5);
            } else {
                controller1.setRumble(RumbleType.kBothRumble, 0);
                controller2.setRumble(RumbleType.kBothRumble, 0);
            }
        }*/


        
        try {

            SmartDashboard.putBoolean("Localization Trustworthy:", drivetrain.isLocalizationTrustworthy());
            SmartDashboard.putNumber("Distance Traveled since last seen apriltag:", drivetrain.distanceTraveledWithoutCorrection());

            RobotPose = drivetrain.getState().Pose;
            SmartDashboard.putString("ROBOT POSE:", "X:" + Functions.round(RobotPose.getX(), 3) + " Y:" + Functions.round(RobotPose.getY(), 3) + " R:" + Functions.round(RobotPose.getRotation().getDegrees(), 3));
            SmartDashboard.putString("TURRET POSE:", Functions.stringifyPose(getTurretPose()));
            


            //double cameraTX = LimelightHelpers.getTX("limelight");


            //SmartDashboard.putNumber("Vision TX", cameraTX);
            //SmartDashboard.putNumber("Vision TA", LimelightHelpers.getTA("limelight"));
            //SmartDashboard.putBoolean("Vision valid Target", LimelightHelpers.getTargetCount("limelight") > 0);
            //SmartDashboard.putBoolean("IS AUTO DRIVING?", autoDriving);
            //SmartDashboard.putNumber("AUTO Driving forward", autoForwardOutput);
            //SmartDashboard.putNumber("AUTO Driving strafe", autoStrafeOutput);
            //SmartDashboard.putNumber("AUTO Driving rotation", autoRotationOutput);
            //SmartDashboard.putNumber("forward", forwardOutput);
            //SmartDashboard.putNumber("strafe", strafeOutput);
            //SmartDashboard.putNumber("rotation", rotationOutput);
            

            BatteryVoltage = RobotController.getBatteryVoltage();
            SmartDashboard.putNumber("BATTERY VOLTAGE:", BatteryVoltage);

            // I still don't think it needs to go back up since almost everything is running constantly anyways
            if (BatteryVoltage < 8 && currentDriveSupplyCurrentLimit > 30 && CurrentLimitTimer.time() > 1) {
                CurrentLimitTimer.reset();
                currentDriveSupplyCurrentLimit -= 20;
                if (currentDriveSupplyCurrentLimit < 30) currentDriveSupplyCurrentLimit = 30;
                drivetrain.setDriveMotorCurrentLimit(currentDriveSupplyCurrentLimit);
            }
            SmartDashboard.putNumber("Drive Motors Supply Current Limit:", currentDriveSupplyCurrentLimit);

        } catch (NullPointerException e) {
            // do nothing
        }


        Settings.keepWithinPerimeter = SmartDashboard.getBoolean("Keep Robot Within Perimeter:", Settings.keepWithinPerimeter);
        
        //update settings
        if (Settings.tuningTelemetryEnabled) {

            /* 
            AutoTargetingSettings.AutoTurningPID.setP(SmartDashboard.getNumber("Tune Auto Turning kP", AutoTargetingSettings.AutoTurningPID.getP()));
            AutoTargetingSettings.AutoTurningPID.setI(SmartDashboard.getNumber("Tune Auto Turning kI", AutoTargetingSettings.AutoTurningPID.getI()));
            AutoTargetingSettings.AutoTurningPID.setD(SmartDashboard.getNumber("Tune Auto Turning kD", AutoTargetingSettings.AutoTurningPID.getD()));
            AutoTargetingSettings.AutoDrivingPID.setP(SmartDashboard.getNumber("Tune Auto Driving kP", AutoTargetingSettings.AutoDrivingPID.getP()));
            AutoTargetingSettings.AutoDrivingPID.setD(SmartDashboard.getNumber("Tune Auto Driving kD", AutoTargetingSettings.AutoDrivingPID.getD()));
            AutoTargetingSettings.AutoDrivingMaxPower = SmartDashboard.getNumber("Tune Auto Driving Max Power", AutoTargetingSettings.AutoDrivingMaxPower);

            AutoTargetingSettings.AutoAimingEnabled = SmartDashboard.getBoolean("Auto Aiming Enabled", AutoTargetingSettings.AutoAimingEnabled);
            AutoTargetingSettings.AutoDrivingEnabled = SmartDashboard.getBoolean("Auto Driving Enabled", AutoTargetingSettings.AutoDrivingEnabled);
            AutoTargetingSettings.AutoDrivingPower = SmartDashboard.getNumber("Auto Driving Power", AutoTargetingSettings.AutoDrivingPower);
            // AutoTargetingSettings.targetPercentageOfVisionBlocked = SmartDashboard.getNumber("Auto target percentage of blocked vision", AutoTargetingSettings.targetPercentageOfVisionBlocked); */

             
            /*SmartDashboard.putString("Pigeon accel", Functions.stringifyTrans(new Translation3d(gyroData.accelX, gyroData.accelY, gyroData.accelZ)));
            SmartDashboard.putString("Pigeon angVel", Functions.stringifyTrans(new Translation3d(gyroData.angVelX, gyroData.angVelY, gyroData.angVelZ)));
            SmartDashboard.putNumber("Pigeon pitch", Functions.round(Math.toDegrees(gyroData.pitch), 2));
            SmartDashboard.putNumber("Pigeon roll", Functions.round(Math.toDegrees(gyroData.roll), 2));
            SmartDashboard.putNumber("Pigeon yaw", Functions.round(Math.toDegrees(gyroData.yaw), 2));*/
            
        }
    }

    public static Pose2d getTurretPose() {
        Translation2d turretTranslation = new Translation2d(0.133349, -0.196850); 
        Pose2d robotPose = drivetrain.getState().Pose;
        double turretAngle = OuttakeSubsystem.getTurretAngle();

        Pose2d ActualTurretPose = new Pose2d(
            robotPose.getTranslation().plus(turretTranslation.rotateBy(new Rotation2d(-turretAngle))), 
            robotPose.getRotation().minus(new Rotation2d(turretAngle))
        );
        
        return ActualTurretPose;
    }

    public static Translation2d getTurretVelocity() { // Field Centric
        Translation2d turretTranslation = new Translation2d(-0.133349, 0.196850); 

        double angVel = drivetrain.getState().Speeds.omegaRadiansPerSecond;

        Translation2d robotVelocity = drivetrain.getFieldCentricVelocity().toTranslation2d();

        Translation2d rotationalVelocity = new Translation2d(-angVel * turretTranslation.getY(), angVel * turretTranslation.getX());

        return robotVelocity.plus(rotationalVelocity.rotateBy(drivetrain.getState().Pose.getRotation()));
    }

    public static Translation3d getTurretAcceleration() { // Field Centric
        Translation3d robotAccel = drivetrain.getFieldCentricAcceleration();
        double angAccel = drivetrain.getAngAcceleration().getRadians(); 

        Translation2d turretTranslation = new Translation2d(-0.133349, 0.196850); 

        double angVel = drivetrain.getState().Speeds.omegaRadiansPerSecond;

        double ax = - angAccel * turretTranslation.getY() - angVel * angVel * turretTranslation.getX();
        double ay = angAccel * turretTranslation.getX() - angVel * angVel * turretTranslation.getY();

        return robotAccel.plus(new Translation3d(ax, ay, 0));
    }

    public void stopRumble() {
        if (haveControllerObjects && runRumble) {
            controller1.setRumble(RumbleType.kBothRumble, 0);
            controller2.setRumble(RumbleType.kBothRumble, 0);
        }
    }

}
