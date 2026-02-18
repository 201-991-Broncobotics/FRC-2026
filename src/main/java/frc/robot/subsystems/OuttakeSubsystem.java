package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Settings.TurretSettings;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.ThroughBoreEncoder;
import frc.robot.utility.Vector2d;
import frc.robot.utility.LimelightHelpers.LimelightResults;

import java.util.Collection;
import java.util.Optional;
import java.util.Vector;
import java.util.function.DoubleSupplier;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;
import org.apache.commons.math3.analysis.polynomials.*;

public class OuttakeSubsystem extends SubsystemBase {

    private TalonFX leftFlyMotor,rightFlyMotor, turntableMotor, hoodMotor;
    private TalonFXConfiguration flywheelConfig, turntableConfig, hoodConfig; 
    private StatusCode flywheelStatus, turntableStatus, hoodStatus; 
    private CurrentLimitsConfigs currentLimits; 
    private double lastkP, lastkI, lastkD, lastkS, lastkV, lastkA, 
                   lastTkP, lastTkI, lastTkD, lastPkP, lastPkI, lastPkD;
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
    private double TurretStartingOffset = 0;

    private ThroughBoreEncoder throughBore21, throughBore19, throughBoreCounter;

    // Only for testing
    private double TargetHoodAngle = 0, TargetTurretAngle = 0, TargetFlywheelVel = 0;

    private DoubleSupplier CurrentTurretAngle, CurrentHoodAngle, CurrentFlywheelRPM;


    public OuttakeSubsystem(CommandSwerveDrivetrain Drivetrain){
        this.drivetrain = Drivetrain;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        distanceVector = new Vector2d();

        if (drivetrain != null) RobotState = drivetrain.getState();
        FrameTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        

        results = LimelightHelpers.getLatestResults(TurretConstants.limelightName);
        if (alliance.get() == Alliance.Red) LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        else if (alliance.get() == Alliance.Blue) LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 1);
            
        throughBore21 = new ThroughBoreEncoder(0);
        throughBore19 = new ThroughBoreEncoder(1);
        throughBoreCounter = new ThroughBoreEncoder(2, 3);






        p1 = new WeightedObservedPoint(1, distanceVector.mag(), 3);
        
        table.add(p1);
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
        function.value(2);

        leftFlyMotor = new TalonFX(MotorConstants.leftFlyID); //follower
        rightFlyMotor = new TalonFX(MotorConstants.rightFlyID); //leader

        turntableMotor = new TalonFX(MotorConstants.turntableID); 

        hoodMotor = new TalonFX(MotorConstants.turretPivotID); 

        flywheelConfig = new TalonFXConfiguration(); 
        turntableConfig = new TalonFXConfiguration(); 
        hoodConfig = new TalonFXConfiguration(); 

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

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        hoodConfig.Voltage.PeakForwardVoltage = TurretConstants.maxForwardVoltage; 
        hoodConfig.Voltage.PeakReverseVoltage = TurretConstants.maxReverseVoltage; 

        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
        hoodConfig.Slot0.kP = TurretSettings.pkP; 
        hoodConfig.Slot0.kI = TurretSettings.pkI; 
        hoodConfig.Slot0.kD = TurretSettings.pkD; 

        //add current limits
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        currentLimits.SupplyCurrentLimit = TurretConstants.supplyCurrent;
        currentLimits.StatorCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        currentLimits.StatorCurrentLimit = TurretConstants.statorCurrent;

        flywheelConfig.CurrentLimits = currentLimits; 
        turntableConfig.CurrentLimits = currentLimits; 
        hoodConfig.CurrentLimits = currentLimits; 

        leftFlyMotor.getConfigurator().apply(flywheelConfig); 
        rightFlyMotor.getConfigurator().apply(flywheelConfig); 

        turntableMotor.getConfigurator().apply(turntableConfig);

        hoodMotor.getConfigurator().apply(hoodConfig);

        flywheelStatus = rightFlyMotor.getConfigurator().apply(flywheelConfig); 
        turntableStatus = turntableMotor.getConfigurator().apply(turntableConfig); 
        hoodStatus = hoodMotor.getConfigurator().apply(hoodConfig); 

        lastkP = TurretSettings.kP;
        lastkI = TurretSettings.kI;
        lastkD = TurretSettings.kD;
        lastkS = TurretSettings.kS;
        lastkV = TurretSettings.kV;
        lastkA = TurretSettings.kA;
        lastTkP = TurretSettings.tkP;
        lastTkI = TurretSettings.tkI; 
        lastTkD = TurretSettings.tkD; 
        lastPkP = TurretSettings.pkP; 
        lastPkI = TurretSettings.pkI; 
        lastPkD = TurretSettings.pkD; 

        if (!flywheelStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Flywheel motors are broken!");
        if (!turntableStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Turntable motor with ID " + MotorConstants.turntableID + " is broken!"); 
        if (!hoodStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Pivot motor with ID " + MotorConstants.turretPivotID + " is broken!"); 

        SmartDashboard.putNumber("Flywheel kP", TurretSettings.kP);
        SmartDashboard.putNumber("Flywheel kI", TurretSettings.kI);
        SmartDashboard.putNumber("Flywheel kD", TurretSettings.kD);
        SmartDashboard.putNumber("Flywheel kS", TurretSettings.kS);
        SmartDashboard.putNumber("Flywheel kV", TurretSettings.kV);
        SmartDashboard.putNumber("Flywheel kA", TurretSettings.kA);
        SmartDashboard.putNumber("Turntable kP", TurretSettings.tkP); 
        SmartDashboard.putNumber("Turntable kI", TurretSettings.tkI); 
        SmartDashboard.putNumber("Turntable kD", TurretSettings.tkD); 
        SmartDashboard.putNumber("Turntable Pivot kP", TurretSettings.pkP);
        SmartDashboard.putNumber("Turntable Pivot kI", TurretSettings.pkI);
        SmartDashboard.putNumber("Turntable Pivot kD", TurretSettings.pkD);
        SmartDashboard.putNumber("Flywheel Speeds", TurretSettings.setVelocities);
        SmartDashboard.putNumber("Predicted Velocity", getFlywheelTrajectory());
        SmartDashboard.putNumber("Turntable Velocity", getTurntableTrajectory()); 


        CurrentTurretAngle = () -> 2 * Math.PI * turntableMotor.getPosition().getValueAsDouble() / 20.0;
        CurrentHoodAngle = () -> hoodMotor.getPosition().getValueAsDouble();
        CurrentFlywheelRPM = () -> 0;

    }

    public void target(double X, double Y, double H) { // h is height

        double Speed = Math.hypot(RobotState.Speeds.vxMetersPerSecond, RobotState.Speeds.vyMetersPerSecond);
        double VelocityAngle = RobotState.Pose.getRotation().getRadians() + Math.atan2(RobotState.Speeds.vyMetersPerSecond, RobotState.Speeds.vxMetersPerSecond);

        Pose2d predictedRobotPose = new Pose2d(
            RobotState.Pose.getX() + FrameTime * Speed * Math.cos(VelocityAngle), 
            RobotState.Pose.getY() + FrameTime * Speed * Math.sin(VelocityAngle), 
            RobotState.Pose.getRotation().plus(new Rotation2d(FrameTime * RobotState.Speeds.omegaRadiansPerSecond))
            );

        
        double Distance = Math.hypot(predictedRobotPose.getX() - X, predictedRobotPose.getY() - Y); // TODO: needs to be from center of turret

        double Velocity = 1; // TODO: Can't use the Distance above because that doesn't incude height which messes up the quadratic regression

        double StartingLaunchHeight = 23; // TODO: get more accurate calculation
        double a = 1 - ((2 * TurretConstants.gravityInches * (H - StartingLaunchHeight)) / Math.pow(Velocity, 2)) - 
            ((Math.pow(TurretConstants.gravityInches, 2) * Math.pow(Distance, 2)) / Math.pow(Velocity, 4));
        
        if (a >= 0) TargetHoodAngle = Math.atan((Math.pow(Velocity, 2) / (TurretConstants.gravityInches * Distance)) * (1 + Math.sqrt(a)));
        else TargetHoodAngle = 0;

        setHood(TargetHoodAngle);

        // TODO: set flywheel rpm based on ball velocity

    }


    public boolean setHood(double Angle) {
        if (Angle >= TurretConstants.minHoodAngle && Angle <= TurretConstants.maxHoodAngle) {
            // TODO: set hood motor
            TargetHoodAngle = Angle;
            return true;
        } else {
            // TODO: set hood motor
            TargetHoodAngle = Functions.minMaxValue(TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, Angle);
            return false;
        }
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
        function.value(getFlywheelTrajectory()); //put this inside motionmagic later 

        rightFlyMotor.setControl(new MotionMagicVelocityDutyCycle(TurretSettings.setVelocities).withSlot(0)); 
        leftFlyMotor.setControl(new Follower(MotorConstants.rightFlyID, MotorAlignmentValue.Opposed)); 
    }

    public void tuneTurntable(){
        turntableMotor.setControl(new MotionMagicVelocityDutyCycle(getTurntableTrajectory())); 
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

        if (TurretSettings.pkP != lastPkP){
            lastPkP = TurretSettings.pkP;
            hoodConfig.Slot0.kP = lastPkP; 
            hoodValueHasChanged = true;  
        }

        if (TurretSettings.pkI != lastPkI){
            lastPkI = TurretSettings.pkI;
            hoodConfig.Slot0.kI = lastPkI; 
            hoodValueHasChanged = true;  
        }

        if (TurretSettings.pkD != lastPkD){
            lastPkD = TurretSettings.pkD;
            hoodConfig.Slot0.kD = lastPkD; 
            hoodValueHasChanged = true;  
        }

        if (flywheelValueHasChanged){
            leftFlyMotor.getConfigurator().apply(flywheelConfig);
            rightFlyMotor.getConfigurator().apply(flywheelConfig);
        }

        if (turntableValueHasChanged) turntableMotor.getConfigurator().apply(turntableConfig); 
        if (hoodValueHasChanged) hoodMotor.getConfigurator().apply(hoodConfig); 

    }


    public double getTurretAngle() { return CurrentTurretAngle.getAsDouble(); }
    public double getHoodAngle() { return CurrentHoodAngle.getAsDouble(); }
    public double getFlywheelRPM() { return CurrentFlywheelRPM.getAsDouble(); }

    private double absoluteTurretAngle() {
        double R = throughBore19.getAbsoluteAngle() - throughBore21.getAbsoluteAngle();
        if (R < 0) R += 2* Math.PI;
        return 0.9975 * R; // fixes angle being slightly off, also means the turret can't keep rotating
    }


    @Override
    public void periodic(){
        FrameTime = FrameTimer.time();
        FrameTimer.reset();

        if (drivetrain != null) {
            RobotState = drivetrain.getState();
        }

        TurretSettings.kP = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kP);
        TurretSettings.kI = SmartDashboard.getNumber("Flywheel kI", TurretSettings.kI);
        TurretSettings.kD = SmartDashboard.getNumber("Flywheel kD", TurretSettings.kD);
        TurretSettings.kS = SmartDashboard.getNumber("Flywheel kS", TurretSettings.kS);
        TurretSettings.kV = SmartDashboard.getNumber("Flywheel kV", TurretSettings.kV);
        TurretSettings.kA = SmartDashboard.getNumber("Flywheel kA", TurretSettings.kA);
        TurretSettings.tkP = SmartDashboard.getNumber("Turntable kP", TurretSettings.tkP); 
        TurretSettings.tkI = SmartDashboard.getNumber("Turntable kI", TurretSettings.tkI);
        TurretSettings.tkI = SmartDashboard.getNumber("Turntable kD", TurretSettings.tkD);
        TurretSettings.pkP = SmartDashboard.getNumber("Turntable Pivot kP", TurretSettings.pkP);
        TurretSettings.pkI = SmartDashboard.getNumber("Turntable Pivot kP", TurretSettings.pkI);
        TurretSettings.pkI = SmartDashboard.getNumber("Turntable Pivot kP", TurretSettings.pkD);
        TurretSettings.setVelocities = SmartDashboard.getNumber("Flywheel Speeds", TurretSettings.setVelocities);

        
        checkForTuning();
    }
    
}
