package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.Settings.OuttakeTrajectorySettings;
import frc.robot.Settings.TurretSettings;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.ThroughBoreEncoder;
import frc.robot.utility.Vector2d;
import frc.robot.utility.ElapsedTime.Resolution;
import frc.robot.utility.LimelightHelpers.LimelightResults;
import frc.robot.utility.Zoning.Zoning;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.Vector;
import java.util.function.DoubleSupplier;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;
import org.apache.commons.math3.analysis.polynomials.*;

public class OuttakeSubsystem extends SubsystemBase {

    private TalonFX leftFlyMotor,rightFlyMotor, turntableMotor;
    private TalonFXConfiguration flywheelConfig, turntableConfig; 
    private final PositionVoltage turretPositionRequest = new PositionVoltage(0);
    private SparkFlex hoodMotor;
    private SparkFlexConfig hoodConfig;
    private SparkClosedLoopController hoodClosedLoopController;
    private StatusCode flywheelStatus, turntableStatus, hoodStatus; 
    private CurrentLimitsConfigs currentLimits; 
    private double lastkP, lastkI, lastkD, lastkS, lastkV, lastkA, 
                   lastTkP, lastTkI, lastTkD, lastTkS, lastTkV, lastPkP, lastPkI, lastPkD;
    private Vector2d distanceVector;  
    private PolynomialCurveFitter regression; 
    private PolynomialFunction function;
    private Collection<WeightedObservedPoint> table; 
    private WeightedObservedPoint 
            p1, p2, p3, p4, p5,
            p6, p7, p8, p9, p10; 
    private LimelightResults results; 
    private SwerveDriveState RobotState;
    private CommandSwerveDrivetrain drivetrain;
    private double FrameTime = 0.1;
    private ElapsedTime FrameTimer;
    private boolean justToggledTuning = false;
    private double TurretStartingOffset = 0;
    private double lastSimSolveTime = 0;
    private double[] lastSimTraj = new double[] {0,0,0,0,0}; // launch angle, launch vel, target dist, target height, flight time
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
    private CommandXboxController controller;

    // I swear to GOD this better make these encoder actually exist
    private static final ThroughBoreEncoder throughBore21 = new ThroughBoreEncoder(0);
    private static final ThroughBoreEncoder throughBore19 = new ThroughBoreEncoder(1);
    private static final ThroughBoreEncoder throughBoreCounter = new ThroughBoreEncoder(2, 3);;

    private double counterAngle = 0;
    private int ballsCounted = 0;

    // Only for testing
    private double TargetHoodAngle = 0, TargetTurretAngle = 0, TargetFlywheelRPM = 0;//Degrees
    private double rpmAdjustment = 50;

    public static DoubleSupplier CurrentTurretAngle, CurrentHoodAngle, CurrentFlywheelRPM;
    private boolean hoodNeedsToBeCalibrated = true, autoLowered = false;
    private int hoodCalibrationPhase = 0;
    private ElapsedTime hoodCalibrationTimer;
    
    private double lowestHoodMotorRev = 0, highestHoodMotorRev = 2.31; // should be just a rough underestimate because it will get retuned when the robot tries to the shoot for the first time
    private boolean IsShooting = false;
    private double turretOffset = 0;

    //temporary
    private double flywheelPower = 0.5;
    private boolean justChangedPower = false;

    private Pose2d robotPose = new Pose2d(0,0, new Rotation2d(0));
    private Zoning FlyZoning = new Zoning(ZoneConstants.TrenchZones);

    public OuttakeSubsystem(CommandSwerveDrivetrain Drivetrain, CommandXboxController operator){
        this.drivetrain = Drivetrain;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        distanceVector = new Vector2d();
        IsShooting = false;

        controller = operator;

        if (drivetrain != null) RobotState = drivetrain.getState();
        FrameTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        hoodCalibrationTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        results = LimelightHelpers.getLatestResults(TurretConstants.limelightName);
        if (alliance.get() == Alliance.Red) LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        else if (alliance.get() == Alliance.Blue) LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        

        




        p1 = new WeightedObservedPoint(1, distanceVector.mag(), 3);
        

        /*table.add(p1);
        table.add(p2);
        table.add(p3);
        table.add(p4);
        table.add(p5);
        table.add(p6);
        table.add(p7);
        table.add(p8);
        table.add(p9);
        table.add(p10);
        
        regression = PolynomialCurveFitter.create(2);

        function = new PolynomialFunction(regression.fit(table));
        function.value(2);*/

        leftFlyMotor = new TalonFX(MotorConstants.leftFlyID); //follower
        rightFlyMotor = new TalonFX(MotorConstants.rightFlyID); //leader

        turntableMotor = new TalonFX(MotorConstants.turntableID); 

        hoodMotor = new SparkFlex(MotorConstants.hoodMotorID, MotorType.kBrushless);

        flywheelConfig = new TalonFXConfiguration(); 
        turntableConfig = new TalonFXConfiguration(); 
        hoodConfig = new SparkFlexConfig();

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        flywheelConfig.Voltage.PeakForwardVoltage = TurretConstants.maxForwardVoltage;
        flywheelConfig.Voltage.PeakReverseVoltage = TurretConstants.maxReverseVoltage; 

        flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
        flywheelConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign; 
        flywheelConfig.Slot0.kP = TurretSettings.kP; 
        flywheelConfig.Slot0.kI = TurretSettings.kI; 
        flywheelConfig.Slot0.kD = TurretSettings.kD; 
        flywheelConfig.Slot0.kS = TurretSettings.kS; 
        flywheelConfig.Slot0.kV = TurretSettings.kV; 
        flywheelConfig.Slot0.kA = TurretSettings.kA; 

        turntableConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        turntableConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turntableConfig.Voltage.PeakForwardVoltage = TurretConstants.maxForwardVoltage; 
        turntableConfig.Voltage.PeakReverseVoltage = TurretConstants.maxReverseVoltage; 

        turntableConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
        turntableConfig.Slot0.kP = TurretSettings.tkP; 
        turntableConfig.Slot0.kI = TurretSettings.tkI; 
        turntableConfig.Slot0.kD = TurretSettings.tkD; 
        turntableConfig.Slot0.kS = TurretSettings.tkS; 
        turntableConfig.Slot0.kV = TurretSettings.tkV; 

        

        //add current limits
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        currentLimits.SupplyCurrentLimit = TurretConstants.supplyCurrent;
        currentLimits.StatorCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        currentLimits.StatorCurrentLimit = TurretConstants.statorCurrent;

        flywheelConfig.CurrentLimits = currentLimits; 
        turntableConfig.CurrentLimits = currentLimits; 

        leftFlyMotor.getConfigurator().apply(flywheelConfig); 
        rightFlyMotor.getConfigurator().apply(flywheelConfig); 

        turntableMotor.getConfigurator().apply(turntableConfig);


        
        // SparkFlex
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        hoodConfig.closedLoop.p(TurretSettings.hkP); 
        hoodConfig.closedLoop.i(TurretSettings.hkI); 
        hoodConfig.closedLoop.d(TurretSettings.hkD); 
        hoodConfig.closedLoop.outputRange(-1, 1);
        hoodConfig.smartCurrentLimit(TurretConstants.hoodMotorCurrent);

        hoodClosedLoopController = hoodMotor.getClosedLoopController();
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelStatus = rightFlyMotor.getConfigurator().apply(flywheelConfig); 
        turntableStatus = turntableMotor.getConfigurator().apply(turntableConfig); 

        lastkP = TurretSettings.kP;
        lastkI = TurretSettings.kI;
        lastkD = TurretSettings.kD;
        lastkS = TurretSettings.kS;
        lastkV = TurretSettings.kV;
        lastkA = TurretSettings.kA;
        lastTkP = TurretSettings.tkP;
        lastTkI = TurretSettings.tkI; 
        lastTkD = TurretSettings.tkD; 
        lastTkS = TurretSettings.tkS; 
        lastTkV = TurretSettings.tkV; 
        lastPkP = TurretSettings.hkP; 
        lastPkI = TurretSettings.hkI; 
        lastPkD = TurretSettings.hkD; 

        if (!flywheelStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Flywheel motors are broken!");
        if (!turntableStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Turntable motor with ID " + MotorConstants.turntableID + " is broken!"); 
        //if (!hoodStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Pivot motor with ID " + MotorConstants.hoodMotorID + " is broken!"); 

        SmartDashboard.putNumber("Flywheel kP", TurretSettings.kP);
        SmartDashboard.putNumber("Flywheel kI", TurretSettings.kI);
        SmartDashboard.putNumber("Flywheel kD", TurretSettings.kD);
        SmartDashboard.putNumber("Flywheel kS", TurretSettings.kS);
        SmartDashboard.putNumber("Flywheel kV", TurretSettings.kV);
        SmartDashboard.putNumber("Flywheel kA", TurretSettings.kA);
        SmartDashboard.putNumber("Turntable kP", TurretSettings.tkP); 
        SmartDashboard.putNumber("Turntable kI", TurretSettings.tkI); 
        SmartDashboard.putNumber("Turntable kD", TurretSettings.tkD); 
        SmartDashboard.putNumber("Turntable kS", TurretSettings.tkS); 
        SmartDashboard.putNumber("Turntable kV", TurretSettings.tkV); 
        SmartDashboard.putNumber("Hood kP", TurretSettings.hkP);
        SmartDashboard.putNumber("Hood kI", TurretSettings.hkI);
        SmartDashboard.putNumber("Hood kD", TurretSettings.hkD);
        SmartDashboard.putNumber("Predicted Velocity", getFlywheelTrajectory());
        SmartDashboard.putNumber("Turntable Velocity", getTurntableTrajectory()); 

        turretOffset = getAbsoluteTurretAngle() - turntableMotor.getPosition().getValueAsDouble() * (2 * Math.PI) / 10.0;
        CurrentTurretAngle = () -> turntableMotor.getPosition().getValueAsDouble() * (2 * Math.PI) / 10.0 + turretOffset; // TODO: needs starting offset
        CurrentHoodAngle = () -> Functions.map(hoodMotor.getEncoder().getPosition(), lowestHoodMotorRev, highestHoodMotorRev, TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle);
        CurrentFlywheelRPM = () -> rightFlyMotor.getVelocity().getValueAsDouble() * 60;

    }


    public void update(){

        // Temporary
        if (controller.povUp().getAsBoolean()) {
            if (!justChangedPower) TurretSettings.setVelocities += rpmAdjustment;
            justChangedPower = true;
        } else if (controller.povDown().getAsBoolean()) {
            if (!justChangedPower) TurretSettings.setVelocities -= rpmAdjustment;
            justChangedPower = true;
        } else justChangedPower = false;

        TargetFlywheelRPM = TurretSettings.setVelocities;

        TargetHoodAngle += MathUtil.applyDeadband(-controller.getRightY(), 0.05) * Math.toRadians(TurretConstants.incrementAngle);
        TargetHoodAngle = Functions.minMaxValue(TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, TargetHoodAngle);

        if (IsShooting) {
            // TURNTABLE
            //temporarily here
            TargetTurretAngle = Functions.minMaxValue(TurretSettings.minTurretAngle, TurretSettings.maxTurretAngle, TargetTurretAngle);
            // setTurntable(TargetTurretAngle); // TODO: still needs starting offset

            // FLYWHEELS
            setFlywheels(TargetFlywheelRPM);


            // HOOD
            if (autoLowered && TurretSettings.autoLowerHood) {
                setHood(TurretConstants.minHoodAngle);

            } else if (hoodNeedsToBeCalibrated) {
                switch (hoodCalibrationPhase) {
                    case 0: // reset timer
                        hoodCalibrationTimer.reset();
                        hoodCalibrationPhase++;
                    case 1: // make sure its hitting bottom mechanical stop
                        hoodMotor.set(-TurretSettings.hoodCalibrationPower);
                        if (hoodCalibrationTimer.time() > TurretSettings.hoodCalibrationDownTime) hoodCalibrationPhase++;
                        break;
                    case 2: // go up to roughly where the top is at pid speed
                        setHood(TurretConstants.maxHoodAngle); // should be underestimate at the moment
                        if (CurrentHoodAngle.getAsDouble() + Math.toRadians(5) > TurretConstants.maxHoodAngle) hoodCalibrationPhase++;
                        hoodCalibrationTimer.reset();
                        break;
                    case 3: // keep going up until it hits the top mechanical stop
                        hoodMotor.set(TurretSettings.hoodCalibrationPower);
                        if (hoodCalibrationTimer.time() > TurretSettings.hoodCalibrationUpTime) hoodCalibrationPhase++;
                        break;
                    case 4: // finialize as the min and max pos get constantly updated in periodic
                        setHood(TargetHoodAngle);
                        hoodNeedsToBeCalibrated = false;
                        hoodCalibrationPhase = 0;
                        break;
                }
            } else {
                // Normal Hood operation
                setHood(TargetHoodAngle);
            }
        } else { // Not shooting
            
            stopFlywheels();
            setHood(TurretConstants.minHoodAngle);
        }

    }

    public void target(double X, double Y, double H) { // h is height

        double Speed = Math.hypot(RobotState.Speeds.vxMetersPerSecond, RobotState.Speeds.vyMetersPerSecond);
        double VelocityAngle = RobotState.Pose.getRotation().getRadians() + Math.atan2(RobotState.Speeds.vyMetersPerSecond, RobotState.Speeds.vxMetersPerSecond);
        // TODO: Velocity is multiplied by the air time of the ball
        Pose2d predictedRobotPose = new Pose2d(
            RobotState.Pose.getX(),// + Speed * Math.cos(VelocityAngle), 
            RobotState.Pose.getY(),// + Speed * Math.sin(VelocityAngle), 
            RobotState.Pose.getRotation()//.plus(new Rotation2d(FrameTime * RobotState.Speeds.omegaRadiansPerSecond))
        );
    
        TargetTurretAngle = Math.atan2(predictedRobotPose.getY() - Y, predictedRobotPose.getX() - X) - predictedRobotPose.getRotation().getRadians();
        
        double Distance = Math.hypot(predictedRobotPose.getX() - X, predictedRobotPose.getY() - Y); // TODO: needs to be from center of turret

        double Velocity = 1; // TODO: Can't use the Distance above because that doesn't incude height which messes up the quadratic regression

        // TODO: Swap with actual calculations
        double StartingLaunchHeight = 23;
        double a = 1 - ((2 * TurretConstants.gravityInches * (H - StartingLaunchHeight)) / Math.pow(Velocity, 2)) - 
            ((Math.pow(TurretConstants.gravityInches, 2) * Math.pow(Distance, 2)) / Math.pow(Velocity, 4));
        
        if (a >= 0) TargetHoodAngle = Math.atan((Math.pow(Velocity, 2) / (TurretConstants.gravityInches * Distance)) * (1 + Math.sqrt(a)));
        else TargetHoodAngle = 0;

        TargetFlywheelRPM = getTargetFlywheelRPM(Velocity);

    }


    public void setTurntable(double Angle) {
        TargetTurretAngle = Functions.minMaxValue(TurretSettings.minTurretAngle, TurretSettings.maxTurretAngle, TargetTurretAngle);
        turntableMotor.setControl(turretPositionRequest.withPosition(TargetTurretAngle / (2*Math.PI) * 10));
    }

    public void setHood(double Angle) {//Radians
        Angle = Functions.minMaxValue(TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, Angle);

        double motorRot = Functions.map(Angle, TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, lowestHoodMotorRev, highestHoodMotorRev);
        hoodClosedLoopController.setSetpoint(motorRot, ControlType.kPosition);
    }

    public void setFlywheels(double rpm) {
        rightFlyMotor.setControl(flywheelVelocityRequest.withVelocity(rpm / 60.0)); 
        leftFlyMotor.setControl(new Follower(MotorConstants.rightFlyID, MotorAlignmentValue.Opposed)); 
    }

    public void stopFlywheels() {
        rightFlyMotor.stopMotor();
        leftFlyMotor.stopMotor();
    }


    public double getTargetFlywheelRPM(double launchVelocity) {
        return 0; // TODO: math
    }

    public double getTargetLaunchVelocity(double targetFlywheelRPM) {
        return 0; // TODO: math
    }

    public double getTargetHoodAngle(double launchAngle) {
        return 0; // TODO: math
    }

    

    public void neutralPosition() {

    }



    public double getFlywheelTrajectory(){
        distanceVector.x = RobotState.Pose.getX();
        distanceVector.y = RobotState.Pose.getY();

        return distanceVector.mag(); 

    }

    public double getTurntableTrajectory() { //copied from limelight documentation
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = -(LimelightHelpers.getTX(TurretConstants.limelightName)/360);
        return targetingAngularVelocity;

    }    

    public void tuneFlywheel(){
        if (!justToggledTuning) {
            IsShooting = true;
            // rightFlyMotor.setControl
            //rightFlyMotor.setControl(flywheelVelocityRequest.withVelocity(TargetFlywheelRPM / 60.0)); 
            //leftFlyMotor.setControl(new Follower(MotorConstants.rightFlyID, MotorAlignmentValue.Opposed)); 
            justToggledTuning = true;
        } else {
            //stopFlywheels();
            IsShooting = false;
            justToggledTuning = false;
        }
    }

    public void tuneTurntable(){
        turntableMotor.setControl(new MotionMagicDutyCycle(TurretSettings.targetTurntableAngle / 360 * 10)); 
    }


    public void checkForTuning(){
        boolean flywheelValueHasChanged = false; 
        boolean turntableValueHasChanged = false; 
        boolean hoodValueHasChanged = false; 

        if (TurretSettings.kP != lastkP){
            lastkP = TurretSettings.kP;
            flywheelConfig.Slot0.kP = lastkP;
            flywheelValueHasChanged = true;
        }

        if (TurretSettings.kI != lastkI){
            lastkI = TurretSettings.kI;
            flywheelConfig.Slot0.kI = lastkI;
            flywheelValueHasChanged = true;
        }

        if (TurretSettings.kD != lastkD){
            lastkD = TurretSettings.kD;
            flywheelConfig.Slot0.kD = lastkD;
            flywheelValueHasChanged = true;
        }

        if (TurretSettings.kS != lastkS){
            lastkS = TurretSettings.kS;
            flywheelConfig.Slot0.kS = lastkS;
            flywheelValueHasChanged = true;
        }

        if (TurretSettings.kV != lastkV){
            lastkV = TurretSettings.kV;
            flywheelConfig.Slot0.kV = lastkV;
            flywheelValueHasChanged = true;
        }

        if (TurretSettings.kA != lastkA){
            lastkA = TurretSettings.kA;
            flywheelConfig.Slot0.kA = lastkA;
            flywheelValueHasChanged = true;
        }

        if (TurretSettings.tkP != lastTkP){
            lastTkP = TurretSettings.tkP;
            turntableConfig.Slot0.kP = lastTkP; 
            turntableValueHasChanged = true;  
        }

        if (TurretSettings.tkI != lastTkI){
            lastTkI = TurretSettings.tkI;
            turntableConfig.Slot0.kI = lastTkI; 
            turntableValueHasChanged = true;  
        }

        if (TurretSettings.tkD != lastTkD){
            lastTkD = TurretSettings.tkD;
            turntableConfig.Slot0.kD = lastTkD; 
            turntableValueHasChanged = true;  
        }

        if (TurretSettings.tkS != lastTkS){
            lastTkS = TurretSettings.tkS;
            turntableConfig.Slot0.kS = lastTkS; 
            turntableValueHasChanged = true;  
        }

        if (TurretSettings.tkV != lastTkV){
            lastTkV = TurretSettings.tkV;
            turntableConfig.Slot0.kV = lastTkV; 
            turntableValueHasChanged = true;  
        }

        if (TurretSettings.hkP != lastPkP){
            lastPkP = TurretSettings.hkP;
            hoodConfig.closedLoop.p(lastPkP); 
            hoodValueHasChanged = true;  
        }

        if (TurretSettings.hkI != lastPkI){
            lastPkI = TurretSettings.hkI;
            hoodConfig.closedLoop.i(lastPkI); 
            hoodValueHasChanged = true;  
        }

        if (TurretSettings.hkD != lastPkD){
            lastPkD = TurretSettings.hkD;
            hoodConfig.closedLoop.d(lastPkD); 
            hoodValueHasChanged = true;  
        }

        if (flywheelValueHasChanged){
            leftFlyMotor.getConfigurator().apply(flywheelConfig);
            rightFlyMotor.getConfigurator().apply(flywheelConfig);
        }

        if (turntableValueHasChanged) turntableMotor.getConfigurator().apply(turntableConfig); 
        if (hoodValueHasChanged) hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }


    public void changeRPMFast() { rpmAdjustment = 500; }
    public void changeRPMSlow() { rpmAdjustment = 50; }

    public void startShooting() { IsShooting = true; }
    public void stopShooting() { IsShooting = true; }


    public static double getTurretAngle() { return CurrentTurretAngle.getAsDouble(); }
    public static double getHoodAngle() { return CurrentHoodAngle.getAsDouble(); }
    public static double getFlywheelRPM() { return CurrentFlywheelRPM.getAsDouble(); }

    public static double getAbsoluteTurretAngle() {
        double R = (throughBore19.getAbsoluteRaw() - throughBore21.getAbsoluteRaw()) * 2*Math.PI;
        if (R < 0) R += 2* Math.PI;
        return -0.9975 * R - TurretSettings.TurretAbsoluteOffset; // fixes angle being slightly off, also means the turret can't keep rotating
    }


    /**
     * Gets the height in meters of the ball when it starts its freefall trajectory or basically when it stops 
     * touching the flywheel and the hood.
     * @param launchAngle radians
     * @return height in meters
     */
    public double getLaunchHeight(double launchAngle) {
        return (4.954998 * Math.cos(launchAngle - Math.toRadians(25.140344)) + 6.649409 + 11.947224) / 39.37;
    }

    /**
     * Gets the distance in meters from the center of the turret of the ball when it starts its freefall trajectory 
     * or basically when it stops touching the flywheel and the hood.
     * @param launchAngle radians
     * @return distance from middle of turret in meters (negative for behind center)
     */
    public double getLaunchForward(double launchAngle) {
        return (-4.954998 * Math.sin(launchAngle - Math.toRadians(25.140344)) + 1.71) / 39.37;
    }

    // REALLY COMPLICATED MATH STUFF - mostly chatgpt

    double solveForAngle(double targetDistance, double targetHeight, double currentLaunchVel, double currentFlywheelRPM) {
        ElapsedTime timer = new ElapsedTime(Resolution.MILLISECONDS);
        timer.reset();

        double low = TurretConstants.minHoodAngle;
        double high = TurretConstants.maxHoodAngle;
        double tolerance = OuttakeTrajectorySettings.SolutionTolerance;

        for (int i = 0; i < 25; i++) {
            double mid = (low + high) / 2.0;

            double yAtX = simulateTrajectory(mid, currentLaunchVel, targetDistance, currentFlywheelRPM * 2*Math.PI/60);

            double error = yAtX - targetHeight;

            if (error > 0) {
                high = mid;
            } else {
                low = mid;
            }

            if (Math.abs(error) < tolerance) {
                break;
            }
        }

        lastSimSolveTime = timer.time();
        return (low + high) / 2.0;
    }

    double solveForAngle(double targetDistance, double targetHeight, double currentFlywheelRPM) {
        return solveForAngle(targetDistance, targetHeight, getTargetLaunchVelocity(currentFlywheelRPM), currentFlywheelRPM);
    }


    double simulateTrajectory(double launchAngle, double launchVelocity, double targetDistance, double flywheelAngVel) { // meters and radians
        double flightTime = 0;
        
        double x = getLaunchForward(launchAngle);
        double y = getLaunchHeight(launchAngle);

        double vx = launchVelocity * Math.cos(launchAngle);
        double vy = launchVelocity * Math.sin(launchAngle);

        double ballSpinOmega = OuttakeTrajectorySettings.SpinTransferEfficiency * (flywheelAngVel * /*roller radius*/(2/39.37) / /*ball radius*/(5.91/39.37/2)); // assumed constant across trajectory

        while (x < targetDistance && y > 0) {
            double v = Math.sqrt(vx*vx + vy*vy);

            double dragAx = -OuttakeTrajectorySettings.KD * v * vx;
            double dragAy = -OuttakeTrajectorySettings.KD * v * vy;

            double liftAx = -OuttakeTrajectorySettings.KL * ballSpinOmega * (5.91 / 39.37 / 2) * vy;
            double liftAy =  OuttakeTrajectorySettings.KL * ballSpinOmega * (5.91 / 39.37 / 2) * vx;

            double ax = dragAx + liftAx;
            double ay = -(9.81) + dragAy + liftAy;

            vx += ax * OuttakeTrajectorySettings.dt;
            vy += ay * OuttakeTrajectorySettings.dt;
            x += vx * OuttakeTrajectorySettings.dt;
            y += vy * OuttakeTrajectorySettings.dt;
            flightTime += OuttakeTrajectorySettings.dt;
        }
        lastSimTraj[4] = flightTime;
        return y;
    }


    public void testRunSim() {
        solveForAngle(OuttakeTrajectorySettings.targetDistance, OuttakeTrajectorySettings.targetHeight, TargetFlywheelRPM);
    }

    @Override
    public void periodic(){
        FrameTime = FrameTimer.time();
        FrameTimer.reset();

        robotPose = drivetrain.getState().Pose;

        ZoneConstants.allianceZone.updateZones(robotPose);
        //if(FlyZoning.ifLeftZones(robotPose) && !IsShooting && ZoneConstants.allianceZone.getZoningState()) IsShooting = true; //If entered the alliance zone and left trench zone, turn on the flywheel and hood

        FlyZoning.updateZones(DrivingProfiles.getTurretPose(robotPose));
        autoLowered = FlyZoning.getZoningState();

        if(FlyZoning.getZoningState() && IsShooting) IsShooting = false; //If entered the trench, stop flywheels + hood

        if (drivetrain != null) {
            RobotState = drivetrain.getState();
        }

        //Auto Lower
        SmartDashboard.putBoolean("BLT Zone", ZoneConstants.blueLeftTrench.inZone(robotPose));
        SmartDashboard.putBoolean("BRT Zone", ZoneConstants.blueRightTrench.inZone(robotPose));
        SmartDashboard.putBoolean("RLT Zone", ZoneConstants.redLeftTrench.inZone(robotPose));
        SmartDashboard.putBoolean("RRT Zone", ZoneConstants.redRightTrench.inZone(robotPose));
        SmartDashboard.putBoolean("Trench Zone",  FlyZoning.inZones(robotPose));
        SmartDashboard.putBoolean("Is Shooting", IsShooting);
        SmartDashboard.putBoolean("Alliance Zone", ZoneConstants.allianceZone.getZoningState());

        TurretSettings.kP = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kP);
        TurretSettings.kI = SmartDashboard.getNumber("Flywheel kI", TurretSettings.kI);
        TurretSettings.kD = SmartDashboard.getNumber("Flywheel kD", TurretSettings.kD);
        TurretSettings.kS = SmartDashboard.getNumber("Flywheel kS", TurretSettings.kS);
        TurretSettings.kV = SmartDashboard.getNumber("Flywheel kV", TurretSettings.kV);
        TurretSettings.kA = SmartDashboard.getNumber("Flywheel kA", TurretSettings.kA);
        TurretSettings.tkP = SmartDashboard.getNumber("Turntable kP", TurretSettings.tkP); 
        TurretSettings.tkI = SmartDashboard.getNumber("Turntable kI", TurretSettings.tkI);
        TurretSettings.tkD = SmartDashboard.getNumber("Turntable kD", TurretSettings.tkD);
        TurretSettings.tkS = SmartDashboard.getNumber("Turntable kS", TurretSettings.tkS);
        TurretSettings.tkV = SmartDashboard.getNumber("Turntable kV", TurretSettings.tk);
        TurretSettings.hkP = SmartDashboard.getNumber("Hood kP", TurretSettings.hkP);
        TurretSettings.hkI = SmartDashboard.getNumber("Hood kI", TurretSettings.hkI);
        TurretSettings.hkD = SmartDashboard.getNumber("Hood kD", TurretSettings.hkD);
        checkForTuning();

        SmartDashboard.putNumber("Outtake RPM", Math.round(rightFlyMotor.getVelocity().getValueAsDouble() * 60));
        SmartDashboard.putBoolean("Auto Lowered?", autoLowered);
        SmartDashboard.putNumber("Flywheel Target RPM", TargetFlywheelRPM);

        try { // prevents crashing

            // Constantly keeps the range up to date since the hood motor cannot physically move outside mechanical stop 
            if (hoodMotor.getEncoder().getPosition() < lowestHoodMotorRev) lowestHoodMotorRev = hoodMotor.getEncoder().getPosition();
            if (hoodMotor.getEncoder().getPosition() > highestHoodMotorRev) highestHoodMotorRev = hoodMotor.getEncoder().getPosition();

            SmartDashboard.putNumber("Flywheel Current RPM", Functions.round(CurrentFlywheelRPM.getAsDouble(), 1));
            SmartDashboard.putNumber("Flywheel Error", Functions.round(TargetFlywheelRPM - CurrentFlywheelRPM.getAsDouble(), 1));
            SmartDashboard.putNumber("Flywheel Motor Temperature", (leftFlyMotor.getDeviceTemp().getValueAsDouble() + rightFlyMotor.getDeviceTemp().getValueAsDouble()) * 0.5);

            SmartDashboard.putNumber("Hood Error (Deg)", Math.toDegrees(TargetHoodAngle - CurrentHoodAngle.getAsDouble()));
            SmartDashboard.putNumber("Hood Target Angle (Deg)", Math.toDegrees(TargetHoodAngle));
            SmartDashboard.putNumber("Hood Angle (Deg)",  Math.toDegrees(CurrentHoodAngle.getAsDouble()));
            SmartDashboard.putNumber("Hood Motor Angle (Rev)", hoodMotor.getEncoder().getPosition());
            
            SmartDashboard.putNumber("ThroughBore 19 Pos:", throughBore19.getAbsoluteAngle());
            SmartDashboard.putNumber("ThroughBore 21 Pos:", throughBore21.getAbsoluteAngle());
            SmartDashboard.putNumber("TEST DISPLAY2:", 2);
            SmartDashboard.putNumber("Turret Relative Position", Functions.round(Math.toDegrees(getTurretAngle()), 2));
            SmartDashboard.putNumber("Turret Absolute Position", Functions.round(Math.toDegrees(getAbsoluteTurretAngle()), 2));
            

            //Counter for the balls - IDK if it gives radians or degrees
            double lastCounterAngle = counterAngle;
            counterAngle = throughBoreCounter.getRelativeAngle();
            //AIDAN & MAEL TAKE A LOOK PLS
            if(!TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) > TurretConstants.counterThreshold && Math.abs(lastCounterAngle) < TurretConstants.counterThreshold)){

                ballsCounted++;
            } else if (TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) < TurretConstants.counterThreshold && Math.abs(lastCounterAngle) > TurretConstants.counterThreshold)){

                ballsCounted++;
            }

            SmartDashboard.putNumber("Last Traj Sim Solve Time (ms)", Math.round(lastSimSolveTime));
            // launch angle, launch vel, target dist, target height, flight time
            SmartDashboard.putString("Last Traj Sim", 
                "angle: " + Functions.round(Math.toDegrees(lastSimTraj[0]), 2) + 
                " vel: " + Functions.round(lastSimTraj[1], 2) + 
                " dist: " + Functions.round(lastSimTraj[2] * 39.37, 2) + 
                " height: " + Functions.round(lastSimTraj[3] * 39.37, 2)
            );
            SmartDashboard.putNumber("Last Traj Sim Flight Time (s)", Functions.round(lastSimTraj[4], 3));

        } catch (NullPointerException e) {
            // do nothing
        }

    }

    public void enableAutoLower(){
        TurretSettings.autoLowerHood = true;
    }

    public void disableAutoLower(){
        TurretSettings.autoLowerHood = false;
    }

    public int getBallsCounted(){
        return ballsCounted;
    }
}
