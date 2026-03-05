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
import com.pathplanner.lib.path.RotationTarget;
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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoDrivingConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.Settings;
import frc.robot.Settings.AutoTargetingSettings;
import frc.robot.Settings.OuttakeTrajectorySettings;
import frc.robot.Settings.Traj;
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
    // private final PositionVoltage turretPositionRequest = new PositionVoltage(0);
    private final MotionMagicVoltage turretPositionRequest = new MotionMagicVoltage(0);
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
    private double lastSimSolveTime = 0;
    private double[] lastSimTraj = new double[] {0,0,0,0,0}; // launch angle, launch vel, target dist, target height, flight time
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

    // Only for testing
    private double TargetHoodAngle = 0, TargetTurretAngle = 0, TargetFlywheelRPM = 0;//Degrees
    private double rpmAdjustment = 50;

    public static DoubleSupplier CurrentTurretAngle, CurrentHoodAngle, CurrentFlywheelRPM;
    private boolean hoodNeedsToBeCalibrated = true, autoLowered = false;
    private int hoodCalibrationPhase = 0;
    private ElapsedTime hoodCalibrationTimer;
    
    private double lowestHoodMotorRev = 0, highestHoodMotorRev = 2.31; // should be just a rough underestimate because it will get retuned when the robot tries to the shoot for the first time
    private boolean IsShooting = false;
    private double turretStartingOffset = 0;

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
        turntableConfig.MotionMagic.MotionMagicCruiseVelocity = 50; // I know its crazy but these actually can be this fast
        turntableConfig.MotionMagic.MotionMagicAcceleration = 75;

        

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

        if (Settings.tuningTelemetryEnabled) {
            /*SmartDashboard.putNumber("Flywheel kP", TurretSettings.kP);
            SmartDashboard.putNumber("Flywheel kI", TurretSettings.kI);
            SmartDashboard.putNumber("Flywheel kD", TurretSettings.kD);
            SmartDashboard.putNumber("Flywheel kS", TurretSettings.kS);
            SmartDashboard.putNumber("Flywheel kV", TurretSettings.kV);
            SmartDashboard.putNumber("Flywheel kA", TurretSettings.kA);*/
            SmartDashboard.putNumber("Turntable kP", TurretSettings.tkP); 
            SmartDashboard.putNumber("Turntable kI", TurretSettings.tkI); 
            SmartDashboard.putNumber("Turntable kD", TurretSettings.tkD); 
            SmartDashboard.putNumber("Turntable kS", TurretSettings.tkS); 
            SmartDashboard.putNumber("Turntable kV", TurretSettings.tkV); 
            SmartDashboard.putNumber("Hood kP", TurretSettings.hkP);
            SmartDashboard.putNumber("Hood kI", TurretSettings.hkI);
            SmartDashboard.putNumber("Hood kD", TurretSettings.hkD);
            SmartDashboard.putNumber("Turntable Velocity", getTurntableTrajectory()); 
        }
        

        turretStartingOffset = getAbsoluteTurretAngle() - (-turntableMotor.getPosition().getValueAsDouble()) * (2 * Math.PI) / 10.0;
        CurrentTurretAngle = () -> (-turntableMotor.getPosition().getValueAsDouble()) * (2 * Math.PI) / 10.0 + turretStartingOffset; // TODO: needs starting offset
        CurrentHoodAngle = () -> Functions.map(hoodMotor.getEncoder().getPosition(), lowestHoodMotorRev, highestHoodMotorRev, TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle);
        CurrentFlywheelRPM = () -> rightFlyMotor.getVelocity().getValueAsDouble() * 60;
        TargetTurretAngle = CurrentTurretAngle.getAsDouble();

    }

    public void start(){
        alliance = DriverStation.getAlliance();

        results = LimelightHelpers.getLatestResults("limelight"); // the limelight name is always limelight unless other cameras which there aren't any
        if (alliance.get() == Alliance.Red) {
            LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        } else if (alliance.get() == Alliance.Blue) {
            LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        }
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

        TargetTurretAngle += MathUtil.applyDeadband(controller.getLeftX(), 0.05) * Math.toRadians(TurretConstants.incrementAngle);
        TargetTurretAngle = Functions.minMaxValue(TurretSettings.minTurretAngle, TurretSettings.maxTurretAngle, TargetTurretAngle);
        

        if (alliance.get() == Alliance.Red) { TARGET = ZoneConstants.redHub;
        } else TARGET = ZoneConstants.blueHub;

        lastTargettingData = getTargettingData(TARGET, 0, 0); // turret, flywheel, hood, air time


        if (IsShooting) {
            
            // Waiting just a sec
            //TargetTurretAngle = lastTargettingData[0];
            //TargetFlywheelRPM = lastTargettingData[1];
            //TargetHoodAngle = lastTargettingData[2];


            // TURNTABLE
            //TargetTurretAngle = getTurretAutoRotation();

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

        setTurntable(TargetTurretAngle);

        /*
        //Auto Aiming
        if(AutoTargetingSettings.AutoAimingEnabled && ZoneConstants.allianceZone.inZones(drivetrain.getState().Pose)){
            setTurntable(getTurretAutoRotation().getRadians());
        } else {
            //Manual Turret Aiming
        }
        */
    }

    public double[] regressTargettingData(Translation3d RelativeTarget, boolean aimHigh) {
        Rotation2d targetRotation = new Rotation2d(RelativeTarget.getX(), RelativeTarget.getY());
        targetRotation = targetRotation.minus(drivetrain.getState().Pose.getRotation());
        double finalTurretAngle = targetRotation.getRadians();
        finalTurretAngle = ((finalTurretAngle - TurretSettings.minTurretAngle)%Math.toRadians(360) + Math.toRadians(360)) % Math.toRadians(360) + TurretSettings.minTurretAngle;


        // TRAJECTORY MATH (Sorry my math was based on old stuff and the tests which are all in inches)
        double Distance = Math.hypot(RelativeTarget.getX(), RelativeTarget.getY());
        double LaunchHeight = 22.3; // INCHES sorry, can also be made to change based on hood angle  
        double finalTargetRPM = (Traj.k1 * (39.3701 * Distance) + Traj.k2) * Math.sqrt((386.088 * Math.pow(39.3701 * Distance, 2)) / ((39.3701 * Distance) - (39.3701 * RelativeTarget.getZ()) + LaunchHeight + Traj.k3));

        // assume minimum necessary flywheel rpm when no where close to the speed needed and end early
        SmartDashboard.putNumber("Traj1 Dist", 39.3701 * Distance);
        SmartDashboard.putNumber("Traj2 Height", 39.3701 * RelativeTarget.getZ());
        SmartDashboard.putNumber("Traj3 RPM", finalTargetRPM);
        if (CurrentFlywheelRPM.getAsDouble() < 500) {
            SmartDashboard.putNumber("Traj4 Hood Angle", 0); // clear data so I can tell when it ends early
            SmartDashboard.putNumber("Traj5 Launch Vel", 0);
            SmartDashboard.putNumber("Traj6 Launch Angle", 0);
            SmartDashboard.putNumber("Traj7 Air Time", 0);
            return new double[]{finalTurretAngle, finalTargetRPM, 0, 0}; 
        }

        // in inches per second, also current means the current target, not based on current position
        double CurrentLaunchVelocity = Traj.a1 * CurrentFlywheelRPM.getAsDouble() + Traj.a2 * Math.pow(CurrentFlywheelRPM.getAsDouble(), 2);
        double EffectiveGravity = 386.088 - Traj.g1 * Math.pow(CurrentLaunchVelocity, 2);
        double CurrentLaunchAngle = 0;
        if (aimHigh) CurrentLaunchAngle = Math.atan((Math.pow(CurrentLaunchVelocity, 2) + Math.sqrt(Math.pow(CurrentLaunchVelocity, 4) + EffectiveGravity*(EffectiveGravity*Math.pow(39.3701 * Distance, 2) + 2*(39.3701 * RelativeTarget.getZ() - LaunchHeight)*Math.pow(CurrentLaunchVelocity, 2)))) / (EffectiveGravity * (39.3701 * Distance)));
        else CurrentLaunchAngle = Math.atan((Math.pow(CurrentLaunchVelocity, 2) - Math.sqrt(Math.pow(CurrentLaunchVelocity, 4) + EffectiveGravity*(EffectiveGravity*Math.pow(39.3701 * Distance, 2) + 2*(39.3701 * RelativeTarget.getZ() - LaunchHeight)*Math.pow(CurrentLaunchVelocity, 2)))) / (EffectiveGravity * (39.3701 * Distance)));

        // double finalHoodAngle = Math.toRadians(((-Traj.b2 + Math.sqrt(Math.pow(Traj.b2, 2) - 4*Traj.b3*(Traj.b1-CurrentLaunchAngle))) / (2*Traj.b3)));
        
        double finalHoodAngle = Math.toRadians(90 - (Traj.m1*(CurrentLaunchAngle)+Traj.m2*(Math.pow(CurrentLaunchAngle, 2)+Traj.m3*(CurrentLaunchVelocity)+Traj.m4*(CurrentLaunchAngle)*(CurrentLaunchVelocity))) + Traj.m5); // i did actually convert launch angle to radians before I regressed this in desmos cause im too lazy to write Math.toRadians() like 5 times, but also I guess I not too lazy to write this extemely long comment at 3am

        double finalAirTime = (39.3701 * Distance) / (CurrentLaunchVelocity * Math.cos(CurrentLaunchAngle));

        
        SmartDashboard.putNumber("Traj4 Hood Angle", Math.toDegrees(finalHoodAngle));
        SmartDashboard.putNumber("Traj5 Launch Vel", CurrentLaunchVelocity);
        SmartDashboard.putNumber("Traj6 Launch Angle", Math.toDegrees(CurrentLaunchAngle));
        SmartDashboard.putNumber("Traj7 Air Time", finalAirTime);

        return new double[]{finalTurretAngle, finalTargetRPM, finalHoodAngle, finalAirTime};

    }
    
    public double[] getTargettingData(Translation3d Target, double TargetForwardOffset, double TargetVerticleOffset, boolean aimHigh) { // TargetForwardOffset is so we can make it always aim for the back half of the goal

        //Get Starting Airtime
        Translation2d offsetTranslation2d = Target.toTranslation2d().minus(DrivingProfiles.getTurretPose(drivetrain.getState().Pose).getTranslation());
        Translation3d relativeGoalPose = new Translation3d(offsetTranslation2d.getX(), offsetTranslation2d.getY(), Target.getZ());
        
        double[] finalRegression = regressTargettingData(relativeGoalPose, aimHigh);
        double airTime = finalRegression[3];
        Translation3d newTarget = Target;
        for (int i = 1; i <= 5; i++) {
            // code that moves relativeGoalPose based on airtime
            Translation2d velocity = new Translation2d(RobotState.Speeds.vxMetersPerSecond, RobotState.Speeds.vyMetersPerSecond);
            Translation2d acceleration = new Translation2d(CommandSwerveDrivetrain.gyroData.accelX, CommandSwerveDrivetrain.gyroData.accelY);

            // d = v*t + 0.5*a*t^2
            Translation2d distanceFromVelocity = velocity.times(airTime);
            Translation2d distanceFromAcceleration = acceleration.times(0.5 * airTime * airTime);
            Translation2d distanceTraveled = distanceFromVelocity.plus(distanceFromAcceleration);

            newTarget = Target.minus(new Translation3d(distanceTraveled));

            offsetTranslation2d = newTarget.toTranslation2d().minus(DrivingProfiles.getTurretPose(drivetrain.getState().Pose).getTranslation());
            relativeGoalPose = new Translation3d(offsetTranslation2d.getX(), offsetTranslation2d.getY(), Target.getZ());

            finalRegression = regressTargettingData(relativeGoalPose, aimHigh);
            airTime = finalRegression[3];
        }

        //Dead Zone
        if(Math.abs(getTurretAngle() - finalRegression[0]) < TurretConstants.AutoTurretDeadband){
            finalRegression[0] = getTurretAngle();
        }
        
        return finalRegression; // Target Turret Angle, Target Flywheel rpm (rough estimate), Target hood angle (based on current flywheel rpm), Estimated Air time
    }


    public double[] getTargettingData(Translation3d Target, double TargetForwardOffset, double TargetVerticleOffset) {
        return getTargettingData(Target, TargetForwardOffset, TargetVerticleOffset, true); // default to high angle solution
    }


    public void setTurntable(double Angle) {
        TargetTurretAngle = Functions.minMaxValue(TurretSettings.minTurretAngle, TurretSettings.maxTurretAngle, TargetTurretAngle);
        // turntableMotor.setControl(turretPositionRequest.withPosition(-((TargetTurretAngle - turretStartingOffset) / (2*Math.PI) * 10)));
        turntableMotor.setControl(new MotionMagicVoltage(-((TargetTurretAngle - turretStartingOffset) / (2*Math.PI) * 10)));
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
    public void stopShooting() { IsShooting = false; }


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

        // 1. Calculate and store poses once to save CPU cycles
        Pose2d robotPose = drivetrain.getState().Pose;
        Pose2d turretPose = DrivingProfiles.getTurretPose(robotPose);

        // 2. Update and store the Alliance Zone state
        ZoneConstants.allianceZone.updateZones(robotPose);
        boolean inAllianceZone = ZoneConstants.allianceZone.getZoningState();

        // 3. Check if we just left the Trench (FlyZoning) BEFORE updating its state
        boolean leftTrenchZone = FlyZoning.ifLeftZones(turretPose);
        SmartDashboard.putBoolean("Left Zone", leftTrenchZone);

        // 4. Logic: Start shooting if we left the trench and are in the alliance zone
        if (leftTrenchZone && !IsShooting && inAllianceZone) {
            IsShooting = true; 
        }

        // 5. Update the Trench (FlyZoning) state
        FlyZoning.updateZones(turretPose);
        autoLowered = FlyZoning.getZoningState(); // autoLowered is true if inside the trench

        // 6. Logic: Stop shooting if we are currently inside the trench
        if (autoLowered && IsShooting) {
            IsShooting = false; 
        }

        if(!ZoneConstants.allianceZone.getZoningState() && IsShooting && TARGET.equals(ZoneConstants.allianceHub)){
            TARGET = new Translation3d(ZoneConstants.allianceZone.getPose2d().getX(), robotPose.getY(), ZoneConstants.allianceHub.getZ());

        } else if (ZoneConstants.allianceZone.getZoningState() && !(TARGET.equals(ZoneConstants.allianceHub))){
            TARGET = ZoneConstants.allianceHub;

        }


        if (drivetrain != null) {
            RobotState = drivetrain.getState();
        }

        SmartDashboard.putBoolean("Is Shooting", IsShooting);
        SmartDashboard.putBoolean("Trench Zone",  FlyZoning.inZones(robotPose));
        SmartDashboard.putBoolean("Alliance Zone", ZoneConstants.allianceZone.getZoningState());

        SmartDashboard.putBoolean("Auto Lowered?", autoLowered);
        

        if (Settings.tuningTelemetryEnabled) {

            //Auto Lower
            SmartDashboard.putBoolean("BLT Zone", ZoneConstants.blueLeftTrench.inZone(robotPose));
            SmartDashboard.putBoolean("BRT Zone", ZoneConstants.blueRightTrench.inZone(robotPose));
            SmartDashboard.putBoolean("RLT Zone", ZoneConstants.redLeftTrench.inZone(robotPose));
            SmartDashboard.putBoolean("RRT Zone", ZoneConstants.redRightTrench.inZone(robotPose));
            

            /*TurretSettings.kP = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kP);
            TurretSettings.kI = SmartDashboard.getNumber("Flywheel kI", TurretSettings.kI);
            TurretSettings.kD = SmartDashboard.getNumber("Flywheel kD", TurretSettings.kD);
            TurretSettings.kS = SmartDashboard.getNumber("Flywheel kS", TurretSettings.kS);
            TurretSettings.kV = SmartDashboard.getNumber("Flywheel kV", TurretSettings.kV);
            TurretSettings.kA = SmartDashboard.getNumber("Flywheel kA", TurretSettings.kA);*/
            TurretSettings.tkP = SmartDashboard.getNumber("Turntable kP", TurretSettings.tkP); 
            TurretSettings.tkI = SmartDashboard.getNumber("Turntable kI", TurretSettings.tkI);
            TurretSettings.tkD = SmartDashboard.getNumber("Turntable kD", TurretSettings.tkD);
            TurretSettings.tkS = SmartDashboard.getNumber("Turntable kS", TurretSettings.tkS);
            TurretSettings.tkV = SmartDashboard.getNumber("Turntable kV", TurretSettings.tkV);
            TurretSettings.hkP = SmartDashboard.getNumber("Hood kP", TurretSettings.hkP);
            TurretSettings.hkI = SmartDashboard.getNumber("Hood kI", TurretSettings.hkI);
            TurretSettings.hkD = SmartDashboard.getNumber("Hood kD", TurretSettings.hkD);
            checkForTuning();

            try { // prevents crashing
                SmartDashboard.putNumber("Hood Error (Deg)", Math.toDegrees(TargetHoodAngle - CurrentHoodAngle.getAsDouble()));
                SmartDashboard.putNumber("Hood Target Angle (Deg)", Math.toDegrees(TargetHoodAngle));
                SmartDashboard.putNumber("Hood Motor Angle (Rev)", hoodMotor.getEncoder().getPosition());

                SmartDashboard.putNumber("Turntable Error (Deg)", Math.toDegrees(TargetTurretAngle) - Functions.round(Math.toDegrees(getAbsoluteTurretAngle()), 2));
                SmartDashboard.putString("TARGET", Functions.stringifyTrans(TARGET));

                SmartDashboard.putNumber("ThroughBore 19 Pos:", throughBore19.getAbsoluteAngle());
                SmartDashboard.putNumber("ThroughBore 21 Pos:", throughBore21.getAbsoluteAngle());
 
            } catch (NullPointerException e) {
                // do nothing
            }


        }

    
        try { // prevents crashing

            // Constantly keeps the range up to date since the hood motor cannot physically move outside mechanical stop 
            if (hoodMotor.getEncoder().getPosition() < lowestHoodMotorRev) lowestHoodMotorRev = hoodMotor.getEncoder().getPosition();
            if (hoodMotor.getEncoder().getPosition() > highestHoodMotorRev) highestHoodMotorRev = hoodMotor.getEncoder().getPosition();

            SmartDashboard.putNumber("Current Hood Angle (Deg)",  Math.toDegrees(CurrentHoodAngle.getAsDouble()));

            SmartDashboard.putNumber("Flywheel Current RPM", Functions.round(CurrentFlywheelRPM.getAsDouble(), 1));
            SmartDashboard.putNumber("Flywheel Target RPM", TargetFlywheelRPM);
            SmartDashboard.putNumber("Flywheel Error", Functions.round(TargetFlywheelRPM - CurrentFlywheelRPM.getAsDouble(), 1));
            SmartDashboard.putNumber("Flywheel Motor Temperature", (leftFlyMotor.getDeviceTemp().getValueAsDouble() + rightFlyMotor.getDeviceTemp().getValueAsDouble()) * 0.5);

            SmartDashboard.putNumber("Turret Relative Position", Functions.round(Math.toDegrees(getTurretAngle()), 2));
            SmartDashboard.putNumber("Turret Absolute Position", Functions.round(Math.toDegrees(getAbsoluteTurretAngle()), 2));
            SmartDashboard.putNumber("Turret Target Position", Math.toDegrees(TargetTurretAngle));
            

            //Counter for the balls - IDK if it gives radians or degrees
            double lastCounterAngle = counterAngle;
            counterAngle = throughBoreCounter.getRelativeAngle();
            //AIDAN & MAEL TAKE A LOOK PLS
            if(!TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) > TurretConstants.counterThreshold && Math.abs(lastCounterAngle) < TurretConstants.counterThreshold)){

                ballsCounted++;
            } else if (TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) < TurretConstants.counterThreshold && Math.abs(lastCounterAngle) > TurretConstants.counterThreshold)){

                ballsCounted++;
            }

            /* 
            SmartDashboard.putNumber("Last Traj Sim Solve Time (ms)", Math.round(lastSimSolveTime));
            // launch angle, launch vel, target dist, target height, flight time
            SmartDashboard.putString("Last Traj Sim", 
                "angle: " + Functions.round(Math.toDegrees(lastSimTraj[0]), 2) + 
                " vel: " + Functions.round(lastSimTraj[1], 2) + 
                " dist: " + Functions.round(lastSimTraj[2] * 39.37, 2) + 
                " height: " + Functions.round(lastSimTraj[3] * 39.37, 2)
            );
            SmartDashboard.putNumber("Last Traj Sim Flight Time (s)", Functions.round(lastSimTraj[4], 3));
            */ 
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
