package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Settings.OuttakeTrajectorySettings;
import frc.robot.Settings.TurretSettings;
import frc.robot.utility.Zone;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.ThroughBoreEncoder;
import frc.robot.utility.Vector2d;
import frc.robot.utility.ElapsedTime.Resolution;
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
    private boolean justToggledTuning = false;
    private double TurretStartingOffset = 0;
    private double lastSimSolveTime = 0;
    private double[] lastSimTraj = new double[] {0,0,0,0,0}; // launch angle, launch vel, target dist, target height, flight time

    private ThroughBoreEncoder throughBore21, throughBore19, throughBoreCounter;

    private double counterAngle = 0;
    private int ballsCounted = 0;

    // Only for testing
    private double TargetHoodAngle = 0, TargetTurretAngle = 0, TargetFlywheelVel = 0;
    private double rpmAdjustment = 50;

    public static DoubleSupplier CurrentTurretAngle, CurrentHoodAngle, CurrentFlywheelRPM;

    //temporary
    private double flywheelPower = 0.5;
    private boolean justChangedPower = false;

    private Pose2d robotPose = new Pose2d(0,0, new Rotation2d(0));
    private boolean autoLowered = false;

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

        hoodMotor = new TalonFX(MotorConstants.hoodMotorID); 

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
        hoodConfig.Slot0.kP = TurretSettings.hkP; 
        hoodConfig.Slot0.kI = TurretSettings.hkI; 
        hoodConfig.Slot0.kD = TurretSettings.hkD; 

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
        lastPkP = TurretSettings.hkP; 
        lastPkI = TurretSettings.hkI; 
        lastPkD = TurretSettings.hkD; 

        if (!flywheelStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Flywheel motors are broken!");
        if (!turntableStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Turntable motor with ID " + MotorConstants.turntableID + " is broken!"); 
        if (!hoodStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Pivot motor with ID " + MotorConstants.hoodMotorID + " is broken!"); 

        SmartDashboard.putNumber("Flywheel kP", TurretSettings.kP);
        SmartDashboard.putNumber("Flywheel kI", TurretSettings.kI);
        SmartDashboard.putNumber("Flywheel kD", TurretSettings.kD);
        SmartDashboard.putNumber("Flywheel kS", TurretSettings.kS);
        SmartDashboard.putNumber("Flywheel kV", TurretSettings.kV);
        SmartDashboard.putNumber("Flywheel kA", TurretSettings.kA);
        SmartDashboard.putNumber("Turntable kP", TurretSettings.tkP); 
        SmartDashboard.putNumber("Turntable kI", TurretSettings.tkI); 
        SmartDashboard.putNumber("Turntable kD", TurretSettings.tkD); 
        SmartDashboard.putNumber("Hood kP", TurretSettings.hkP);
        SmartDashboard.putNumber("Hood kI", TurretSettings.hkI);
        SmartDashboard.putNumber("Hood kD", TurretSettings.hkD);
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

        // TODO: Swap with actual calculations
        double StartingLaunchHeight = 23;
        double a = 1 - ((2 * TurretConstants.gravityInches * (H - StartingLaunchHeight)) / Math.pow(Velocity, 2)) - 
            ((Math.pow(TurretConstants.gravityInches, 2) * Math.pow(Distance, 2)) / Math.pow(Velocity, 4));
        
        if (a >= 0) TargetHoodAngle = Math.atan((Math.pow(Velocity, 2) / (TurretConstants.gravityInches * Distance)) * (1 + Math.sqrt(a)));
        else TargetHoodAngle = 0;

        setHood(TargetHoodAngle);

        rightFlyMotor.setControl(new MotionMagicVelocityDutyCycle(getTargetFlywheelRPM(Velocity) / 60.0).withSlot(0)); 
        leftFlyMotor.setControl(new Follower(MotorConstants.rightFlyID, MotorAlignmentValue.Opposed)); 

    }


    public boolean setHood(double Angle) {
        return setHood(Angle, true);
    }

    public boolean setHood(double Angle, boolean saveAngle) {
        boolean inRange = (Angle >= TurretConstants.minHoodAngle && Angle <= TurretConstants.maxHoodAngle);

        if (!inRange){//Constraint to hood range
            Angle = Functions.minMaxValue(TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle, Angle);
        }

        if (saveAngle) {//Save the target hood angle to angle if you wanna save
            TargetHoodAngle = Angle;
        }

        if (!autoLowered) { //Set hood angle to constrained angle
            hoodMotor.setControl(new MotionMagicVelocityDutyCycle(Angle)); 
        }

        return inRange;
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
        //function.value(getFlywheelTrajectory()); //put this inside motionmagic later 
        if (!justToggledTuning) {
            rightFlyMotor.setControl(new MotionMagicVelocityDutyCycle(TurretSettings.setVelocities / 60.0).withSlot(0)); 
            leftFlyMotor.setControl(new Follower(MotorConstants.rightFlyID, MotorAlignmentValue.Opposed)); 
            justToggledTuning = true;
        } else {
            stopFlywheels();
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

        if (TurretSettings.hkP != lastPkP){
            lastPkP = TurretSettings.hkP;
            hoodConfig.Slot0.kP = lastPkP; 
            hoodValueHasChanged = true;  
        }

        if (TurretSettings.hkI != lastPkI){
            lastPkI = TurretSettings.hkI;
            hoodConfig.Slot0.kI = lastPkI; 
            hoodValueHasChanged = true;  
        }

        if (TurretSettings.hkD != lastPkD){
            lastPkD = TurretSettings.hkD;
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

    public void justRunFlywheel() {
        rightFlyMotor.set(flywheelPower);
        leftFlyMotor.set(-flywheelPower);
    }

    public void stopFlywheels() {
        rightFlyMotor.stopMotor();
        leftFlyMotor.stopMotor();
    }

    public void increaseFlywheelPower() {
        /*if (!justChangedPower) flywheelPower += 0.1;
        justChangedPower = true;
        if (flywheelPower > 1) flywheelPower = 1;*/

        if (!justChangedPower) TurretSettings.setVelocities += rpmAdjustment;
        justChangedPower = true;
    }

    public void decreaseFlywheelPower() {
        /*
        if (!justChangedPower) flywheelPower -= 0.1;
        justChangedPower = true;
        if (flywheelPower < -1) flywheelPower = -1;*/

        if (!justChangedPower) TurretSettings.setVelocities -= rpmAdjustment;
        justChangedPower = true;
    }

    public void unclickFlywheelPower() {
        justChangedPower = false;
    }

    public void changeRPMFast() {
        rpmAdjustment = 500;
    }

    public void changeRPMSlow() {
        rpmAdjustment = 50;
    }


    public double getTurretAngle() { return CurrentTurretAngle.getAsDouble(); }
    public double getHoodAngle() { return CurrentHoodAngle.getAsDouble(); }
    public double getFlywheelRPM() { return CurrentFlywheelRPM.getAsDouble(); }

    private double getAbsoluteTurretAngle() {
        double R = throughBore19.getAbsoluteAngle() - throughBore21.getAbsoluteAngle();
        if (R < 0) R += 2* Math.PI;
        return 0.9975 * R; // fixes angle being slightly off, also means the turret can't keep rotating
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
        solveForAngle(OuttakeTrajectorySettings.targetDistance, OuttakeTrajectorySettings.targetHeight, TurretSettings.setVelocities);
    }

    @Override
    public void periodic(){
        FrameTime = FrameTimer.time();
        FrameTimer.reset();

        if (drivetrain != null) {
            RobotState = drivetrain.getState();
        }

        //Auto Lower
        Pose2d lastRobotPose = robotPose;
        robotPose = drivetrain.getState().Pose;

        if(TurretSettings.autoLowerHood && DrivingProfiles.ifEnteredAreas(robotPose, lastRobotPose, TurretSettings.Zones)){
            //Lower hood and dont save the angle as the target
            setHood(TurretConstants.minHoodAngle, false);
            autoLowered = true;
        }else if (TurretSettings.autoLowerHood &&DrivingProfiles.ifLeftAreas(robotPose, lastRobotPose, TurretSettings.Zones)){
            autoLowered = false;
            //Lift hood back up to set angle
            setHood(TargetHoodAngle);
        }
        

        //Counter for the balls - IDK if it gives radians or degrees
        double lastCounterAngle = counterAngle;
        counterAngle = throughBoreCounter.getRelativeAngle();

        //AIDAN & MAEL TAKE A LOOK PLS
        if(!TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) > TurretConstants.counterThreshold && Math.abs(lastCounterAngle) < TurretConstants.counterThreshold)){

            ballsCounted++;
        } else if (TurretSettings.reverseCounterDirection && (Math.abs(counterAngle) < TurretConstants.counterThreshold && Math.abs(lastCounterAngle) > TurretConstants.counterThreshold)){

            ballsCounted++;
        }

        TurretSettings.kP = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kP);
        TurretSettings.kI = SmartDashboard.getNumber("Flywheel kI", TurretSettings.kI);
        TurretSettings.kD = SmartDashboard.getNumber("Flywheel kD", TurretSettings.kD);
        TurretSettings.kS = SmartDashboard.getNumber("Flywheel kS", TurretSettings.kS);
        TurretSettings.kV = SmartDashboard.getNumber("Flywheel kV", TurretSettings.kV);
        TurretSettings.kA = SmartDashboard.getNumber("Flywheel kA", TurretSettings.kA);
        TurretSettings.tkP = SmartDashboard.getNumber("Turntable kP", TurretSettings.tkP); 
        TurretSettings.tkI = SmartDashboard.getNumber("Turntable kI", TurretSettings.tkI);
        TurretSettings.tkD = SmartDashboard.getNumber("Turntable kD", TurretSettings.tkD);
        TurretSettings.hkP = SmartDashboard.getNumber("Hood kP", TurretSettings.hkP);
        TurretSettings.hkI = SmartDashboard.getNumber("Hood kI", TurretSettings.hkI);
        TurretSettings.hkD = SmartDashboard.getNumber("Hood kD", TurretSettings.hkD);
        TurretSettings.setVelocities = SmartDashboard.getNumber("Flywheel Speeds", TurretSettings.setVelocities);
        checkForTuning();

        try { // prevents crashing
            SmartDashboard.putNumber("Turret Absolute Position", Math.toDegrees(getAbsoluteTurretAngle()));
            SmartDashboard.putNumber("Turret Relative Position", Math.toDegrees(getTurretAngle()));
            SmartDashboard.putNumber("Flywheel Motor Temperature", (leftFlyMotor.getDeviceTemp().getValueAsDouble() + rightFlyMotor.getDeviceTemp().getValueAsDouble()) * 0.5);
        } catch (NullPointerException e) {
            // do nothing
        }

        SmartDashboard.putNumber("Outtake RPM", Math.round(rightFlyMotor.getVelocity().getValueAsDouble() * 60));


        try { // prevents crashing
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
