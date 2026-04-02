package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TraverseConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.Settings;
import frc.robot.Settings.RobotSettings;
import frc.robot.Settings.Traj;
import frc.robot.Settings.TurretSettings;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.ThroughBoreEncoder;
import frc.robot.utility.Vector2d;
import frc.robot.utility.LimelightHelpers.LimelightResults;
import frc.robot.utility.Zoning.Zoning;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class OuttakeSubsystem extends SubsystemBase {

    public static enum States {
        Off,
        AutoLowered,
        AutoTargetingHub,
        AutoTargetingAllianceZone
    }

    public static States states = States.Off;

    private TalonFX leftFlyMotor,rightFlyMotor, turntableMotor;
    private TalonFXConfiguration flywheelConfig, turntableConfig; 
    // private final PositionVoltage turretPositionRequest = new PositionVoltage(0);
    private final MotionMagicVoltage turretPositionRequest = new MotionMagicVoltage(0);
    private SparkFlex hoodMotor;
    private SparkFlexConfig hoodConfig;
    private SparkClosedLoopController hoodClosedLoopController;
    private StatusCode flywheelStatus, turntableStatus, hoodStatus; 
    private CurrentLimitsConfigs currentLimits, turntableCurrentLimits; 
    private double lastkP, lastkI, lastkD, lastkS, lastkV, lastkA, 
                   lastTkP, lastTkI, lastTkD, lastTkS, lastTkV, lastPkP, lastPkI, lastPkD;
    private Vector2d distanceVector;  
    //private LimelightResults results; 
    private SwerveDriveState RobotState;
    private CommandSwerveDrivetrain drivetrain;
    private double FrameTime = 0.1;
    private ElapsedTime FrameTimer;
    private boolean justToggledTuning = false;
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
    private CommandXboxController controller;
    private Optional<Alliance> alliance;
    private Translation3d TARGET = ZoneConstants.allianceHub;
    private double[] lastTargettingData = new double[]{0, 0, 0, 0};

    // I swear to GOD this better make these encoder actually exist
    private static final ThroughBoreEncoder throughBore21 = new ThroughBoreEncoder(0);
    private static final ThroughBoreEncoder throughBore19 = new ThroughBoreEncoder(1);
    private static final ThroughBoreEncoder throughBoreCounter = new ThroughBoreEncoder(2, 3);;

    private double counterAngle = 0;
    private int ballsCounted = 0;
    private boolean currentSpiking = false;

    // Only for testing
    private double TargetHoodAngle = 0, TargetTurretAngle = 0, TargetFlywheelRPM = 0;//Degrees
    private double rpmAdjustment = 50;

    public static DoubleSupplier CurrentTurretAngle, CurrentHoodAngle, CurrentFlywheelRPM;
    private boolean hoodNeedsToBeCalibrated = true, autoLowered = false;
    private int hoodCalibrationPhase = 0;
    private ElapsedTime hoodCalibrationTimer;
    
    private double lowestHoodMotorRev = 0, highestHoodMotorRev = 2.31; // should be just a rough underestimate because it will get retuned when the robot tries to the shoot for the first time
    private boolean Shooting = false;
    private double turretStartingOffset = 0;

    private boolean justChangedPower = false, loweredMaxCurrent = false;

    private Pose2d robotPose = new Pose2d(0,0, new Rotation2d(0));
    private Zoning FlyZoning = new Zoning(ZoneConstants.TrenchZones);

    private boolean dumbShooterMode = false, antiAirMode = false, localizationMode = false;

    
    private ArrayList<Double> averageTurntableAngle = new ArrayList(), averageFlywheelRPM = new ArrayList(), averageHoodAngle = new ArrayList();
    private double[] averageData = new double[]{0, 0, 0};


    public static boolean TurretWillMiss = true;


    public OuttakeSubsystem(CommandSwerveDrivetrain Drivetrain, CommandXboxController operator){
        this.drivetrain = Drivetrain;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        distanceVector = new Vector2d();
        Shooting = false;

        controller = operator;

        if (drivetrain != null) RobotState = drivetrain.getState();
        FrameTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        hoodCalibrationTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        averageTurntableAngle.add(0.0);
        averageFlywheelRPM.add(0.0);
        averageHoodAngle.add(0.0);
        
        ballsCounted = 0;

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
        turntableConfig.MotionMagic.MotionMagicCruiseVelocity = 50; // I know its crazy but these actually can be this fast
        turntableConfig.MotionMagic.MotionMagicAcceleration = 75;

        

        //add current limits
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        currentLimits.SupplyCurrentLimit = TurretConstants.supplyCurrent;
        currentLimits.StatorCurrentLimitEnable = false; //TurretConstants.currentLimitsEnabled;
        currentLimits.StatorCurrentLimit = TurretConstants.statorCurrent;

        turntableCurrentLimits = new CurrentLimitsConfigs();
        turntableCurrentLimits.SupplyCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        turntableCurrentLimits.SupplyCurrentLimit = TurretConstants.turretSupplyCurrent;
        turntableCurrentLimits.StatorCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        turntableCurrentLimits.StatorCurrentLimit = TurretConstants.turretStatorCurrent;

        flywheelConfig.CurrentLimits = currentLimits; 
        turntableConfig.CurrentLimits = turntableCurrentLimits; 

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

        if (Settings.tuningTelemetryEnabled) {
            /*SmartDashboard.putNumber("Flywheel kP", TurretSettings.kP);
            SmartDashboard.putNumber("Flywheel kI", TurretSettings.kI);
            SmartDashboard.putNumber("Flywheel kD", TurretSettings.kD);
            SmartDashboard.putNumber("Flywheel kS", TurretSettings.kS);
            SmartDashboard.putNumber("Flywheel kV", TurretSettings.kV);
            SmartDashboard.putNumber("Flywheel kA", TurretSettings.kA);
            
            SmartDashboard.putNumber("Hood kP", TurretSettings.hkP);
            SmartDashboard.putNumber("Hood kI", TurretSettings.hkI);
            SmartDashboard.putNumber("Hood kD", TurretSettings.hkD);
            SmartDashboard.putNumber("Turntable Velocity", getTurntableTrajectory()); 
            SmartDashboard.putNumber("Turntable kP", TurretSettings.tkP); 
            SmartDashboard.putNumber("Turntable kI", TurretSettings.tkI); 
            SmartDashboard.putNumber("Turntable kD", TurretSettings.tkD); 
            SmartDashboard.putNumber("Turntable kS", TurretSettings.tkS); 
            SmartDashboard.putNumber("Turntable kV", TurretSettings.tkV); */
        }
        

        turretStartingOffset = Math.toRadians(180)/*getAbsoluteTurretAngle()*/ - (-turntableMotor.getPosition().getValueAsDouble()) * (2 * Math.PI) / 10.0;
        CurrentTurretAngle = () -> (-turntableMotor.getPosition().getValueAsDouble()) * (2 * Math.PI) / 10.0 + turretStartingOffset;
        CurrentHoodAngle = () -> Functions.map(hoodMotor.getEncoder().getPosition(), lowestHoodMotorRev, highestHoodMotorRev, TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle);
        CurrentFlywheelRPM = () -> rightFlyMotor.getVelocity().getValueAsDouble() * 60;
        TargetTurretAngle = CurrentTurretAngle.getAsDouble();

    }

    public void start(){
        alliance = DriverStation.getAlliance();

        /*// results = LimelightHelpers.getLatestResults("limelight"); 
        if (alliance.get() == Alliance.Red) {
            LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        } else if (alliance.get() == Alliance.Blue) {
            LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        }*/ // limelights are only used for localization 

        if (alliance.get() == Alliance.Red) { TARGET = ZoneConstants.redHub;
        } else TARGET = ZoneConstants.blueHub;
    }








    public void update(){
        /* 
        double hoodControl = -controller.getRightY();
        double turretControl = controller.getLeftX();

        if(!RobotSettings.overrideMode){
            // CONTROLLS
            if (controller.povUp().getAsBoolean()) {
                if (!justChangedPower) TurretSettings.setVelocities += rpmAdjustment;
                justChangedPower = true;
            } else if (controller.povDown().getAsBoolean()) {
                if (!justChangedPower) TurretSettings.setVelocities -= rpmAdjustment;
                justChangedPower = true;
            } else justChangedPower = false;
        } else {
            //Override Controls
            if (controller.povRight().getAsBoolean()) {
                if (!justChangedPower) TurretSettings.setVelocities += rpmAdjustment;
                justChangedPower = true;
            } else if (controller.povLeft().getAsBoolean()) {
                if (!justChangedPower) TurretSettings.setVelocities -= rpmAdjustment;
                justChangedPower = true;
            } else justChangedPower = false;

            //turretControl = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
        }


        // TargetFlywheelRPM = TurretSettings.setVelocities;

        TargetHoodAngle += MathUtil.applyDeadband(hoodControl, 0.05) * Math.toRadians(TurretConstants.incrementAngle);
        TargetHoodAngle = Functions.minMaxValue(TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, TargetHoodAngle);

        TargetTurretAngle += MathUtil.applyDeadband(turretControl, 0.05) * Math.toRadians(TurretConstants.incrementAngle);
        TargetTurretAngle = Functions.minMaxValue(TurretSettings.minTurretAngle, TurretSettings.maxTurretAngle, TargetTurretAngle);
        */
        
        lastTargettingData = getTargettingData(TARGET, 0.2, 0); // turret, flywheel, hood, air time

        averageTurntableAngle.add(lastTargettingData[0]);
        averageFlywheelRPM.add(lastTargettingData[1]);
        averageHoodAngle.add(lastTargettingData[2]);
        if (averageTurntableAngle.size() > TurretSettings.numberOfIterations) {
            averageTurntableAngle.remove(0);
            averageFlywheelRPM.remove(0);
            averageHoodAngle.remove(0);
        }

        averageData = new double[]{
            averageTurntableAngle.stream().mapToDouble(Double::doubleValue).average().orElse(0.0),
            averageFlywheelRPM.stream().mapToDouble(Double::doubleValue).average().orElse(0.0),
            averageHoodAngle.stream().mapToDouble(Double::doubleValue).average().orElse(0.0),
        };

        //SmartDashboard.putNumber("Average Turntable Angle:", Math.toDegrees(averageData[0]));
        //SmartDashboard.putNumber("Average Flywheel RPM:", averageData[1]);
        //SmartDashboard.putNumber("Average Hood Angle:", Math.toDegrees(averageData[2]));


        if (Shooting && IntakeSubsystem.states != IntakeSubsystem.States.Up) {

            //DrivingProfiles.allowedToUseLimelight = false;
            if (TurretSettings.tuningMode) {
                TargetFlywheelRPM = TurretSettings.setVelocities;
            } else if (dumbShooterMode) {
                TargetTurretAngle = Math.toRadians(180);
                TargetFlywheelRPM = 2532;
                TargetHoodAngle = lastTargettingData[2];
            } else if (localizationMode) { // only aim at april tags
                if (Math.abs(CurrentTurretAngle.getAsDouble() - averageData[0]) > TurretSettings.TurntableDeadband) TargetTurretAngle = averageData[0];
                stopFlywheels();
                TargetHoodAngle = TurretConstants.minHoodAngle;
            } else if (antiAirMode) {
                if (Math.abs(CurrentTurretAngle.getAsDouble() - averageData[0]) > TurretSettings.TurntableDeadband) TargetTurretAngle = averageData[0];
                TargetFlywheelRPM = 5000;
                TargetHoodAngle = 40;
            } else {

                // TURNTABLE
                if (Math.abs(CurrentTurretAngle.getAsDouble() - averageData[0]) > TurretSettings.TurntableDeadband) TargetTurretAngle = averageData[0];

                // Initial flywheel
                if (Math.abs(CurrentFlywheelRPM.getAsDouble() - averageData[1]) > TurretSettings.FlywheelDeadband) TargetFlywheelRPM = averageData[1];
                // if (Math.abs(CurrentHoodAngle.getAsDouble() - averageData[2]) > TurretSettings.HoodDeadband) TargetHoodAngle = averageData[2];
                TargetHoodAngle = lastTargettingData[2]; // hood can be as precise as possible
            }

            // FLYWHEELS
            setFlywheels(TargetFlywheelRPM);

            // HOOD
            if (autoLowered && TurretSettings.autoLowerHood && !TurretSettings.tuningMode) {
                setHood(TurretConstants.minHoodAngle);
            } else if (hoodNeedsToBeCalibrated) {
                switch (hoodCalibrationPhase) {
                    case 0: // reset timer
                        hoodCalibrationTimer.reset();
                        hoodCalibrationPhase++;
                    case 1: // make sure its hitting bottom mechanical stop
                        hoodMotor.set(-TurretSettings.hoodCalibrationPower);
                        if (hoodCalibrationTimer.time() > TurretSettings.hoodCalibrationDownTime) hoodCalibrationPhase = 3;
                        break;
                    case 2: // go up to roughly where the top is at pid speed

                        // Skip this cause its really slow for some reason

                        setHood(TurretConstants.maxHoodAngle); // should be underestimate at the moment
                        if (CurrentHoodAngle.getAsDouble() + Math.toRadians(10) > TurretConstants.maxHoodAngle) hoodCalibrationPhase++;
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

            setTurntable(TargetTurretAngle);

            TurretWillMiss = (TargetTurretAngle < TurretSettings.minTurretAngle || TargetTurretAngle > TurretSettings.maxTurretAngle) || autoLowered || hoodNeedsToBeCalibrated || localizationMode || (Math.abs(TargetFlywheelRPM - CurrentFlywheelRPM.getAsDouble()) > 400) || (Math.abs(TargetTurretAngle - CurrentTurretAngle.getAsDouble()) > Math.toRadians(45));
            
        } else { // Not shooting
            
            stopFlywheels();
            setHood(TurretConstants.minHoodAngle);
            hoodCalibrationPhase = 0;
            TurretWillMiss = true;
            
        }


        /*
        //Auto Aiming
        if(AutoTargetingSettings.AutoAimingEnabled && ZoneConstants.allianceZone.inZones(drivetrain.getState().Pose)){
            setTurntable(getTurretAutoRotation().getRadians());
        } else {
            //Manual Turret Aiming
        }
        */
    }





    
    // ==================================================================================================
    // MATH AND TRAJECTORY CALCULATIONS
    // ==================================================================================================

    public double[] regressTargettingData(Translation3d RelativeTarget, boolean aimHigh, double distanceOffset, double heightOffset) {
        Rotation2d targetRotation = new Rotation2d(RelativeTarget.getX(), RelativeTarget.getY());
        targetRotation = targetRotation.minus(drivetrain.getState().Pose.getRotation());
        double finalTurretAngle = targetRotation.getRadians();

        finalTurretAngle = ((finalTurretAngle - TurretSettings.minTurretAngle)%Math.toRadians(360) + Math.toRadians(360)) % Math.toRadians(360) + TurretSettings.minTurretAngle;

        SmartDashboard.putNumber("Traj0 Turret", Math.toDegrees(finalTurretAngle));


        // TRAJECTORY MATH (Sorry my math was based on old stuff and the tests which are all in inches)
        double Distance = Math.hypot(RelativeTarget.getX(), RelativeTarget.getY()) + distanceOffset;
        double Height = RelativeTarget.getZ() + heightOffset;
        double LaunchHeight = 22.3; // INCHES sorry, can also be made to change based on hood angle  
        double finalTargetRPM = Traj.k1 * ((39.3701 * Distance) + Traj.k2) * Math.sqrt((386.088 * Math.pow(39.3701 * Distance, 2)) / ((39.3701 * Distance) - (39.3701 * Height) + LaunchHeight + Traj.k3));

        if (Double.isNaN(finalTargetRPM)) finalTargetRPM = 0;

        double currentFlywheelRPM = finalTargetRPM; //CurrentFlywheelRPM.getAsDouble();

        // assume minimum necessary flywheel rpm when no where close to the speed needed and end early
        SmartDashboard.putNumber("Traj1 Dist", 39.3701 * Distance);
        SmartDashboard.putNumber("Traj2 Height", 39.3701 * Height);
        SmartDashboard.putNumber("Traj3 RPM", finalTargetRPM);
        if (CurrentFlywheelRPM.getAsDouble() < 1000) {
            SmartDashboard.putNumber("Traj4 Hood Angle", 0); // clear data so I can tell when it ends early
            SmartDashboard.putNumber("Traj5 Launch Vel", 0);
            SmartDashboard.putNumber("Traj6 Launch Angle", 0);
            SmartDashboard.putNumber("Traj7 Air Time", 0);
            return new double[]{finalTurretAngle, finalTargetRPM, 0, 0}; 
        }

        // in inches per second, also current means the current target, not based on current position
        double CurrentLaunchVelocity = Traj.a1 * currentFlywheelRPM + Traj.a2 * Math.pow(currentFlywheelRPM, 2);
        double EffectiveGravity = 386.088 - Traj.g1 * Math.pow(CurrentLaunchVelocity, 2);
        double CurrentLaunchAngle = 0;
        if (aimHigh) CurrentLaunchAngle = Math.atan((Math.pow(CurrentLaunchVelocity, 2) + Math.sqrt(Math.pow(CurrentLaunchVelocity, 4) - EffectiveGravity*(EffectiveGravity*Math.pow(39.3701 * Distance, 2) + 2*(39.3701 * Height - LaunchHeight)*Math.pow(CurrentLaunchVelocity, 2)))) / (EffectiveGravity * (39.3701 * Distance)));
        else CurrentLaunchAngle = Math.atan((Math.pow(CurrentLaunchVelocity, 2) - Math.sqrt(Math.pow(CurrentLaunchVelocity, 4) - EffectiveGravity*(EffectiveGravity*Math.pow(39.3701 * Distance, 2) + 2*(39.3701 * Height - LaunchHeight)*Math.pow(CurrentLaunchVelocity, 2)))) / (EffectiveGravity * (39.3701 * Distance)));

        // double finalHoodAngle = Math.toRadians(((-Traj.b2 + Math.sqrt(Math.pow(Traj.b2, 2) - 4*Traj.b3*(Traj.b1-CurrentLaunchAngle))) / (2*Traj.b3)));
        
        double finalHoodAngle = Math.toRadians(90 - (Traj.m1*(CurrentLaunchAngle)+Traj.m2*(Math.pow(CurrentLaunchAngle, 2))+Traj.m3*(CurrentLaunchVelocity)+Traj.m4*(CurrentLaunchAngle)*(CurrentLaunchVelocity)) + Traj.m5); // i did actually convert launch angle to radians before I regressed this in desmos cause im too lazy to write Math.toRadians() like 5 times, but also I guess I not too lazy to write this extemely long comment at 3am

        double finalAirTime = (39.3701 * Distance) / (CurrentLaunchVelocity * Math.cos(CurrentLaunchAngle));

        if (Double.isNaN(CurrentLaunchAngle) || Double.isInfinite(CurrentLaunchAngle) || Double.isNaN(finalHoodAngle) || Double.isInfinite(finalHoodAngle)) {
            CurrentLaunchAngle = 0;
            finalHoodAngle = TurretConstants.minHoodAngle;
            finalAirTime = 0;
        }

        
        SmartDashboard.putNumber("Traj4 Hood Angle", Math.toDegrees(finalHoodAngle));
        SmartDashboard.putNumber("Traj5 Launch Vel", CurrentLaunchVelocity);
        SmartDashboard.putNumber("Traj6 Launch Angle", Math.toDegrees(CurrentLaunchAngle));
        SmartDashboard.putNumber("Traj7 Air Time", finalAirTime);

        return new double[]{finalTurretAngle, finalTargetRPM, finalHoodAngle, finalAirTime};

    }
    
    public double[] getTargettingData(Translation3d Target, double TargetDistanceOffset, double TargetVerticleOffset, boolean aimHigh) { // TargetForwardOffset is so we can make it always aim for the back half of the goal

        //Get Starting Airtime
        Translation2d offsetTranslation2d = Target.toTranslation2d().minus(DrivingProfiles.getTurretPose().getTranslation());
        Translation3d relativeGoalPose = new Translation3d(offsetTranslation2d.getX(), offsetTranslation2d.getY(), Target.getZ());
        
        //SmartDashboard.putString("Relative Goal Pose", Functions.stringifyTrans(relativeGoalPose));

        double[] finalRegression = regressTargettingData(relativeGoalPose, aimHigh, TargetDistanceOffset, TargetVerticleOffset);
        double airTime = finalRegression[3];
        Translation3d newTarget = Target;
        for (int i = 1; i <= 7; i++) {
            // code that moves relativeGoalPose based on airtime
            Translation2d velocity = DrivingProfiles.getTurretVelocity();
            Translation2d acceleration = DrivingProfiles.getTurretAcceleration().toTranslation2d();

            velocity = velocity.plus(acceleration.times(FrameTime)); // btw the ball doesn't keep accelerating with the robots accel after it leaves the robot, so technically we can only use it to estimate what the velocity will be in the next frame
            Translation2d distanceFromVelocity = velocity.times(airTime);

            newTarget = Target.minus(new Translation3d(distanceFromVelocity));

            offsetTranslation2d = newTarget.toTranslation2d().minus(DrivingProfiles.getTurretPose().getTranslation());
            relativeGoalPose = new Translation3d(offsetTranslation2d.getX(), offsetTranslation2d.getY(), Target.getZ());

            finalRegression = regressTargettingData(relativeGoalPose, aimHigh, TargetDistanceOffset, TargetVerticleOffset);
            airTime = finalRegression[3];
        }

        SmartDashboard.putNumber("TargettingData final airtime", airTime);
        SmartDashboard.putString("Relative Goal Pose", Functions.stringifyTrans(relativeGoalPose));
        
        return finalRegression; // Target Turret Angle, Target Flywheel rpm (rough estimate), Target hood angle (based on current flywheel rpm), Estimated Air time
    }


    public double[] getTargettingData(Translation3d Target, double TargetForwardOffset, double TargetVerticleOffset) {
        return getTargettingData(Target, TargetForwardOffset, TargetVerticleOffset, true); // default to high angle solution
    }



    


    // ==================================================================================================
    // MECHANISM CONTROLS
    // ==================================================================================================

    public void setTurntable(double Angle) {

        // stop moving the turret when outside range
        if (Angle <= TurretSettings.maxTurretAngle && Angle >= TurretSettings.minTurretAngle) {
            Angle = Functions.minMaxValue(TurretSettings.minTurretAngle, TurretSettings.maxTurretAngle, TargetTurretAngle);
            // turntableMotor.setControl(turretPositionRequest.withPosition(-((TargetTurretAngle - turretStartingOffset) / (2*Math.PI) * 10)));
            turntableMotor.setControl(new MotionMagicVoltage(-((Angle - turretStartingOffset) / (2*Math.PI) * 10)));
        }
        
    }

    public void setHood(double Angle) {//Radians
        Angle = Functions.minMaxValue(TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, Angle);

        double motorRot = Functions.map(Angle, TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, lowestHoodMotorRev, highestHoodMotorRev);
        SmartDashboard.putNumber("Hood set target angle", Angle);
        SmartDashboard.putNumber("Hood set target Motor Rot", motorRot);
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

    public double getTurntableTrajectory() { //copied from limelight documentation
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = -(LimelightHelpers.getTX(TurretConstants.limelightName)/360);
        return targetingAngularVelocity;

    }    

    public void changeRPMFast() { rpmAdjustment = 500; }
    public void changeRPMSlow() { rpmAdjustment = 50; }

    public void startShooting() { 
        Shooting = true; 
        Settings.useRLimelight = true;
        Settings.useLLimelight = true;
    }
    public void stopShooting() { 
        Shooting = false; 
        update();
    }
    public void toggleShooting(){
        Shooting = !Shooting;
        /* 
        if (!justToggledTuning) {
            Shooting = true;
            // rightFlyMotor.setControl
            //rightFlyMotor.setControl(flywheelVelocityRequest.withVelocity(TargetFlywheelRPM / 60.0)); 
            //leftFlyMotor.setControl(new Follower(MotorConstants.rightFlyID, MotorAlignmentValue.Opposed)); 
            justToggledTuning = true;
        } else {
            //stopFlywheels();
            Shooting = false;
            justToggledTuning = false;
        }*/
    }

    public void enableFocusMode() { localizationMode = true; }
    public void disableFocusMode() { localizationMode = false; }

    public void enableAutoLower(){ TurretSettings.autoLowerHood = true; }
    public void disableAutoLower(){ TurretSettings.autoLowerHood = false; }

    public void toggleDumbShooter() { dumbShooterMode = !dumbShooterMode; }

    public void enableAntiAir() { antiAirMode = true; }
    public void disableAntiAir() { antiAirMode = false; }

    public void setController(CommandXboxController newController){ controller = newController; }




    
    // ==================================================================================================
    // TUNING
    // ==================================================================================================

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



    
    // ==================================================================================================
    // STATUS METHODS
    // ==================================================================================================

    public static double getTurretAngle() { return CurrentTurretAngle.getAsDouble(); }
    public static double getHoodAngle() { return CurrentHoodAngle.getAsDouble(); }
    public static double getFlywheelRPM() { return CurrentFlywheelRPM.getAsDouble(); }

    public int getBallsCounted(){ return ballsCounted; }

    public static double getAbsoluteTurretAngle() {
        double R = (-throughBore19.getAbsoluteRaw() - -throughBore21.getAbsoluteRaw()) * 2*Math.PI;
        if (R < 0) R += 2* Math.PI;
        return Functions.wrapAngleAround(-0.9975 * R - TurretSettings.TurretAbsoluteOffset, Math.toRadians(-90)); // fixes angle being slightly off, also means the turret can't keep rotating
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

    public Translation2d calculateTargetForHub(Translation2d c, Translation2d q, double tolerance) {
        return calculateTargetForHub(c, q, tolerance, false);
    }

     /**
     * Calculates the point L using Translation2D inputs.
     * Includes a safety check for points inside the radius.
     * * @param c The center point of the hub
     * @param q The shooting point
     * @param tolerance The tolerance of the circle
     * @param alignWithHub Align the end point with the same X as the target
     * @return A new Translation2D representing point L, or null if Q is inside R
     */
    public Translation2d calculateTargetForHub(Translation2d c, Translation2d q, double tolerance, boolean alignWithTarget) {
        double a; //Alliance 1 is Red, -1 is Blue
        if(ZoneConstants.alliance){
            a = -1;
        } else {
            a = 1;
        }

        double r = 1.13383893709 + (tolerance);//Calculated in the desmos

        Translation2d relativeCoords = c.minus(q);
        double dist = Math.sqrt(relativeCoords.getX() * relativeCoords.getX() + relativeCoords.getY() * relativeCoords.getY());

        // Safety Check: acos(x) is undefined if x > 1
        if (dist < r + 0.15) {
            dist = r + 0.15; // Or handle as an error/default to q
        }

        // F(C, Q, R) componentsh
        double thetaX = Math.atan2(relativeCoords.getY(), relativeCoords.getX());
        double thetaY = Math.acos(r / dist);

        // {C.y > 0 : 1, -1}
        double sign = (relativeCoords.getY() > 0) ? -1.0 : 1.0;

        // Combined Angle P(..., F(...))
        double phi = thetaX + (a * sign * thetaY);

        // Resulting Point L
        double lx, ly;

        if(alignWithTarget){
            lx = c.getX();
            ly = c.getX() * (Math.tan(phi - Math.toRadians(90)));
        } else {
            lx = c.getX() + r * Math.cos(phi);
            ly = c.getY() + r * Math.sin(phi);
        }

        return new Translation2d(lx, ly);
    }

    @Override
    public void periodic(){
        FrameTime = FrameTimer.time();
        FrameTimer.reset();

        // 1. Calculate and store poses once to save CPU cycles
        Pose2d robotPose = drivetrain.getState().Pose;
        Pose2d turretPose = DrivingProfiles.getTurretPose();

        if(TurretSettings.autoLowerHood){
            // 4. Logic: Stop shooting if we left the trench and are not in the alliance zone
            if ((FlyZoning.ifLeftZones(turretPose) && Shooting && !ZoneConstants.allianceZone.inZones(turretPose)) || FlyZoning.ifEnteredZones(turretPose) && Shooting && ZoneConstants.allianceZone.inZones(turretPose)) {
                Shooting = false;
            } else if ((FlyZoning.ifEnteredZones(turretPose) && !Shooting && !ZoneConstants.allianceZone.inZones(turretPose))) {
                Shooting = true; 
            }
        }

        // 5. Update the Trench (FlyZoning) state
        FlyZoning.updateZones(turretPose);
        ZoneConstants.allianceZone.updateZones(robotPose);
        autoLowered = FlyZoning.getZoningState(); // autoLowered is true if inside the trench

        //Change targeting
        if(!ZoneConstants.allianceZone.getZoningState()){ // if not in alliance zone

            alliance = DriverStation.getAlliance();
            if (antiAirMode && (ZoneConstants.blueZone.inZone(robotPose) || ZoneConstants.redZone.inZone(robotPose))) { // if still in an alliance zone after checking if not in our alliance zone
                SmartDashboard.putString("Successfully Activated Anti-Air", "yes");
                if (alliance.get() == Alliance.Red) { TARGET = ZoneConstants.blueHub; // Target other hub
                } else TARGET = ZoneConstants.redHub;
            }

            Translation2d aimPoint = calculateTargetForHub(ZoneConstants.allianceHub.toTranslation2d(), turretPose.getTranslation(), 0.25);
            //Uses alliance hub as the regression already accounts for height
            TARGET = new Translation3d(aimPoint.getX(), aimPoint.getY(), ZoneConstants.allianceHub.getZ());
        } else if (ZoneConstants.allianceZone.getZoningState() && !(TARGET.equals(ZoneConstants.allianceHub))){
            TARGET = ZoneConstants.allianceHub;
        }


        if (drivetrain != null) RobotState = drivetrain.getState();

        SmartDashboard.putBoolean("Is Shooting", Shooting);
        SmartDashboard.putBoolean("Trench Zone",  FlyZoning.inZones(robotPose));
        SmartDashboard.putBoolean("Alliance Zone", ZoneConstants.allianceZone.getZoningState());

        SmartDashboard.putBoolean("Auto Lowered?", autoLowered);
        SmartDashboard.putBoolean("Anti air?", antiAirMode);
        

        if (Settings.tuningTelemetryEnabled) {
            


            //Auto Lower
            //SmartDashboard.putBoolean("BLT Zone", ZoneConstants.blueLeftTrench.inZone(DrivingProfiles.getTurretPose()));
            //SmartDashboard.putBoolean("BRT Zone", ZoneConstants.blueRightTrench.inZone(DrivingProfiles.getTurretPose()));
            //SmartDashboard.putBoolean("RLT Zone", ZoneConstants.redLeftTrench.inZone(DrivingProfiles.getTurretPose()));
            //SmartDashboard.putBoolean("RRT Zone", ZoneConstants.redRightTrench.inZone(DrivingProfiles.getTurretPose()));
            

            /*TurretSettings.kP = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kP);
            TurretSettings.kI = SmartDashboard.getNumber("Flywheel kI", TurretSettings.kI);
            TurretSettings.kD = SmartDashboard.getNumber("Flywheel kD", TurretSettings.kD);
            TurretSettings.kS = SmartDashboard.getNumber("Flywheel kS", TurretSettings.kS);
            TurretSettings.kV = SmartDashboard.getNumber("Flywheel kV", TurretSettings.kV);
            TurretSettings.kA = SmartDashboard.getNumber("Flywheel kA", TurretSettings.kA);
            
            TurretSettings.hkP = SmartDashboard.getNumber("Hood kP", TurretSettings.hkP);
            TurretSettings.hkI = SmartDashboard.getNumber("Hood kI", TurretSettings.hkI);
            TurretSettings.hkD = SmartDashboard.getNumber("Hood kD", TurretSettings.hkD);
            
            TurretSettings.tkP = SmartDashboard.getNumber("Turntable kP", TurretSettings.tkP); 
            TurretSettings.tkI = SmartDashboard.getNumber("Turntable kI", TurretSettings.tkI);
            TurretSettings.tkD = SmartDashboard.getNumber("Turntable kD", TurretSettings.tkD);
            TurretSettings.tkS = SmartDashboard.getNumber("Turntable kS", TurretSettings.tkS);
            TurretSettings.tkV = SmartDashboard.getNumber("Turntable kV", TurretSettings.tkV);*/
            checkForTuning();
        }

    
        try { // prevents crashing
            if (DrivingProfiles.currentDriveSupplyCurrentLimit < 50 && !loweredMaxCurrent) {
                currentLimits.SupplyCurrentLimit = TurretConstants.supplyCurrent * 0.65;

                flywheelConfig.CurrentLimits = currentLimits; 
                leftFlyMotor.getConfigurator().apply(flywheelConfig); 
                rightFlyMotor.getConfigurator().apply(flywheelConfig); 

                loweredMaxCurrent = true;
            }

            SmartDashboard.putNumber("Flywheel Motor Current", rightFlyMotor.getTorqueCurrent().getValueAsDouble());
            if (rightFlyMotor.getTorqueCurrent().getValueAsDouble() > 15 && (Math.abs(TargetFlywheelRPM - CurrentFlywheelRPM.getAsDouble()) < 400)) {
                if (!currentSpiking) ballsCounted++;
                currentSpiking = true;
            } else if (rightFlyMotor.getTorqueCurrent().getValueAsDouble() < 10) currentSpiking = false;

            SmartDashboard.putNumber("Total Balls Launched", ballsCounted);

            // Constantly keeps the range up to date since the hood motor cannot physically move outside mechanical stop 
            if (hoodMotor.getEncoder().getPosition() < lowestHoodMotorRev) lowestHoodMotorRev = hoodMotor.getEncoder().getPosition();
            if (hoodMotor.getEncoder().getPosition() > highestHoodMotorRev) highestHoodMotorRev = hoodMotor.getEncoder().getPosition();

            SmartDashboard.putNumber("Current Hood Angle (Deg)",  Math.toDegrees(CurrentHoodAngle.getAsDouble()));
            SmartDashboard.putNumber("Hood Error (Deg)", Math.toDegrees(TargetHoodAngle - CurrentHoodAngle.getAsDouble()));
            SmartDashboard.putNumber("Hood Target Angle (Deg)", Math.toDegrees(TargetHoodAngle));
            SmartDashboard.putNumber("Hood Motor Angle (Rev)", hoodMotor.getEncoder().getPosition());

            // SmartDashboard.putNumber("Turntable Error (Deg)", Math.toDegrees(TargetTurretAngle) - Functions.round(Math.toDegrees(getAbsoluteTurretAngle()), 2));
            SmartDashboard.putString("Shooter TARGET", Functions.stringifyTrans(TARGET));

            SmartDashboard.putString("Middle Aim", Functions.stringifyTrans(calculateTargetForHub(ZoneConstants.allianceHub.toTranslation2d(), turretPose.getTranslation(), 0.25)));

            SmartDashboard.putNumber("ThroughBore 19 Pos:", throughBore19.getAbsoluteAngle());
            SmartDashboard.putNumber("ThroughBore 21 Pos:", throughBore21.getAbsoluteAngle());
            

            SmartDashboard.putNumber("Flywheel Current RPM", Functions.round(CurrentFlywheelRPM.getAsDouble(), 1));
            SmartDashboard.putNumber("Flywheel Target RPM", TargetFlywheelRPM);
            // SmartDashboard.putNumber("Flywheel Error", Functions.round(TargetFlywheelRPM - CurrentFlywheelRPM.getAsDouble(), 1));
            // SmartDashboard.putNumber("Flywheel Motor Temperature", (leftFlyMotor.getDeviceTemp().getValueAsDouble() + rightFlyMotor.getDeviceTemp().getValueAsDouble()) * 0.5);

            SmartDashboard.putNumber("Turret Relative Position", Functions.round(Math.toDegrees(getTurretAngle()), 2));
            SmartDashboard.putNumber("Turret Absolute Position", Functions.round(Math.toDegrees(getAbsoluteTurretAngle()), 2));
            SmartDashboard.putNumber("Turret Target Position", Math.toDegrees(TargetTurretAngle));
            

            //Counter for the balls - IDK if it gives radians or degrees
            double lastCounterAngle = counterAngle;
            counterAngle = throughBoreCounter.getRelativeAngle();
            if(!TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) > TurretConstants.counterThreshold && Math.abs(lastCounterAngle) < TurretConstants.counterThreshold)){

                ballsCounted++;
            } else if (TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) < TurretConstants.counterThreshold && Math.abs(lastCounterAngle) > TurretConstants.counterThreshold)){

                ballsCounted++;
            }

        } catch (NullPointerException e) {
            // do nothing
        }

    }

    
}
