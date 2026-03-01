package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Constants.AutoDrivingConstants;
import frc.robot.Settings.AutoTargetingSettings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain.gyroData;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
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

    private DoubleSupplier fowardControllerInput, strafeControllerInput, rotationControllerInput, throttleControllerInput;

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

    public boolean allowedToUseLimelight = true;


    public DrivingProfiles(CommandSwerveDrivetrain drivetrain) {
        DrivingProfiles.drivetrain = drivetrain;

        allowedToUseLimelight = true;

        RobotPose = drivetrain.getState().Pose;

        FPSTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        if (Settings.tuningTelemetryEnabled) {

            SmartDashboard.putNumber("Tune Auto Turning kP", AutoTargetingSettings.AutoTurningPID.getP());
            SmartDashboard.putNumber("Tune Auto Turning kI", AutoTargetingSettings.AutoTurningPID.getI());
            SmartDashboard.putNumber("Tune Auto Turning kD", AutoTargetingSettings.AutoTurningPID.getD());
            SmartDashboard.putNumber("Tune Auto Driving kP", AutoTargetingSettings.AutoDrivingPID.getP());
            SmartDashboard.putNumber("Tune Auto Driving kD", AutoTargetingSettings.AutoDrivingPID.getD());
            SmartDashboard.putNumber("Tune Auto Driving Max Power", AutoTargetingSettings.AutoDrivingMaxPower);

            SmartDashboard.putBoolean("Auto Aiming Enabled", AutoTargetingSettings.AutoAimingEnabled);
            SmartDashboard.putBoolean("Auto Driving Enabled", AutoTargetingSettings.AutoDrivingEnabled);
            SmartDashboard.putNumber("Auto Driving Power", AutoTargetingSettings.AutoDrivingPower);
            SmartDashboard.putNumber("Auto target percentage of blocked vision", AutoTargetingSettings.targetPercentageOfVisionBlocked);
        }

        LimelightHelpers.setPipelineIndex("limelight", 0);

    }


    public void setUpControllerInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, double driveCurveMag, double turnCurveMag) {
        this.fowardControllerInput = fowardInput;
        this.strafeControllerInput = strafeInput;
        this.rotationControllerInput = rotationInput;
        this.throttleControllerInput = throttleInput;
        this.controllerDriveCurveMag = driveCurveMag;
        this.controllerTurnCurveMag = turnCurveMag;
    }


    public void update() {
        if (updateController());
        else stopDriving();

        //if (useAutoDrivingThrottle) autoDriving = (AutoDrivingThrottle.getAsDouble() > AutoThrottleDeadband);
        if (autoDriving) updateAutoDriving();

    }

// 6/7/1956 - TODO: Fix. This is temporary. - John Maxwell
// 2/16/2026 - This is was NOT temporary.

    private boolean updateController() {
        double forward = fowardControllerInput.getAsDouble();
        double strafe = strafeControllerInput.getAsDouble();
        double turn = rotationControllerInput.getAsDouble();
        double throttle = throttleControllerInput.getAsDouble();

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Math.hypot(forward, strafe);
        double drivePower = Functions.throttleCurve(joystickPower, controllerDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, controllerTurnCurveMag) * throttle;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
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

    public void setupAutoDrivingThrottle(DoubleSupplier AutoThrottle) {
        //useAutoDrivingThrottle = true;
        //AutoDrivingThrottle = AutoThrottle;
    }


    @Override
    public void periodic() {
        if (FPSTimer != null) {
            if (FPSTimer.time() > 0) SmartDashboard.putNumber("FPS:", Functions.round(1.0 / FPSTimer.time(), 2));
            FPSTimer.reset();
        }
        
        if (drivetrain != null && OuttakeSubsystem.CurrentTurretAngle != null) {
            PoseEstimate LimelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if (LimelightPoseEstimate != null) {
                double TurretAngle = OuttakeSubsystem.CurrentTurretAngle.getAsDouble();
                Rotation2d CorrectFacingDirection = LimelightPoseEstimate.pose.getRotation().minus(new Rotation2d(TurretAngle));
                Pose2d OffsetLimelightPose2d = new Pose2d( // 0.163027 meters forward from center of turret, 0.456593 meters above the ground, 15 degee pitch up
                    LimelightPoseEstimate.pose.getX() + 0.237765 * Math.cos(CorrectFacingDirection.getRadians() + Math.toRadians(55.885527)), 
                    LimelightPoseEstimate.pose.getY() + 0.237765 * Math.sin(CorrectFacingDirection.getRadians() + Math.toRadians(55.885527)), 
                    CorrectFacingDirection
                );


                if (LimelightHelpers.validPoseEstimate(LimelightPoseEstimate) && allowedToUseLimelight) drivetrain.addVisionMeasurement(OffsetLimelightPose2d, LimelightPoseEstimate.timestampSeconds, VecBuilder.fill(0.25, 0.25, 5.0)); // standard deviation of vision measurements in meters and degrees
            }
            

            RobotPose = drivetrain.getState().Pose;
            SmartDashboard.putString("ROBOT POSE:", "X:" + Functions.round(RobotPose.getX(), 3) + " Y:" + Functions.round(RobotPose.getY(), 3) + " R:" + Functions.round(RobotPose.getRotation().getDegrees(), 3));

            double cameraTX = LimelightHelpers.getTX("limelight");


            SmartDashboard.putNumber("Vision TX", cameraTX);
            SmartDashboard.putNumber("Vision TA", LimelightHelpers.getTA("limelight"));
            SmartDashboard.putBoolean("Vision valid Target", LimelightHelpers.getTargetCount("limelight") > 0);
            SmartDashboard.putBoolean("IS AUTO DRIVING?", autoDriving);
            SmartDashboard.putNumber("AUTO Driving forward", autoForwardOutput);
            SmartDashboard.putNumber("AUTO Driving strafe", autoStrafeOutput);
            SmartDashboard.putNumber("AUTO Driving rotation", autoRotationOutput);
            SmartDashboard.putNumber("forward", forwardOutput);
            SmartDashboard.putNumber("strafe", strafeOutput);
            SmartDashboard.putNumber("rotation", rotationOutput);
            if (fowardControllerInput != null) {
                SmartDashboard.putNumber("joystick forward", fowardControllerInput.getAsDouble());
                SmartDashboard.putNumber("joystick strafe", strafeControllerInput.getAsDouble());
                SmartDashboard.putNumber("joystick rotation", rotationControllerInput.getAsDouble());
                SmartDashboard.putNumber("joystick magnitude", Math.hypot(fowardControllerInput.getAsDouble(), strafeControllerInput.getAsDouble()));

            }
            

        }
        

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

            SmartDashboard.putNumber("Pigeon accel X", gyroData.accelX);
            SmartDashboard.putNumber("Pigeon accel Y", gyroData.accelY);
            SmartDashboard.putNumber("Pigeon accel Z", gyroData.accelZ);
            SmartDashboard.putNumber("Pigeon pitch", gyroData.pitch);
            SmartDashboard.putNumber("Pigeon roll", gyroData.roll);
            
            SmartDashboard.putNumber("Pigeon angVel X", gyroData.angVelX);
            SmartDashboard.putNumber("Pigeon angVel Y", gyroData.angVelY);
            SmartDashboard.putNumber("Pigeon angVel Z", gyroData.angVelZ);
        }

        
        
        
        
    }
}
