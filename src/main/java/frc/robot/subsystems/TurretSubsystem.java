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

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Settings.TurretSettings;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.Vector2d;
import frc.robot.utility.LimelightHelpers.LimelightResults;

import java.util.Collection;
import java.util.Optional;
import java.util.Vector;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;
import org.apache.commons.math3.analysis.polynomials.*;

public class TurretSubsystem extends SubsystemBase {

    private TalonFX leftFlyMotor,rightFlyMotor;
    private TalonFXConfiguration flywheelConfig; 
    private StatusCode flywheelStatus; 
    private CurrentLimitsConfigs currentLimits; 
    private double lastkP, lastkI, lastkD, lastkS, lastkV, lastkA, x, y;
    private Vector2d distanceVector;  
    private PolynomialCurveFitter regression; 
    private PolynomialFunction function;
    private Collection<WeightedObservedPoint> table; 
    private WeightedObservedPoint 
            p1, p2, p3, p4, p5,
            p6, p7, p8, p9, p10; 
    private LimelightResults results; 


    public TurretSubsystem(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        distanceVector = new Vector2d(x, y);

        results = LimelightHelpers.getLatestResults(TurretConstants.limelightName);
        if (alliance.get() == Alliance.Red) LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 0);
        else if (alliance.get() == Alliance.Blue) LimelightHelpers.setPipelineIndex(TurretConstants.limelightName, 1);
            
        
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

        //add current limits
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        currentLimits.SupplyCurrentLimit = TurretConstants.supplyCurrent;
        currentLimits.StatorCurrentLimitEnable = TurretConstants.currentLimitsEnabled;
        currentLimits.StatorCurrentLimit = TurretConstants.statorCurrent;

        flywheelConfig.CurrentLimits = currentLimits; 

        leftFlyMotor.getConfigurator().apply(flywheelConfig); 
        rightFlyMotor.getConfigurator().apply(flywheelConfig); 

        flywheelStatus = rightFlyMotor.getConfigurator().apply(flywheelConfig); 

        lastkP = TurretSettings.kP;
        lastkI = TurretSettings.kI;
        lastkD = TurretSettings.kD;
        lastkS = TurretSettings.kS;
        lastkV = TurretSettings.kV;
        lastkA = TurretSettings.kA;

        if (!flywheelStatus.isOK()) System.out.println("Flywheel Motors are broken!");

        SmartDashboard.putNumber("Flywheel kP", TurretSettings.kP);
        SmartDashboard.putNumber("Flywheel kI", TurretSettings.kI);
        SmartDashboard.putNumber("Flywheel kD", TurretSettings.kD);
        SmartDashboard.putNumber("Flywheel kS", TurretSettings.kS);
        SmartDashboard.putNumber("Flywheel kV", TurretSettings.kV);
        SmartDashboard.putNumber("Flywheel kA", TurretSettings.kA);
        SmartDashboard.putNumber("Flywheel Speeds", TurretSettings.setVelocities);
        SmartDashboard.putNumber("Predicted Velocity", getTrajectory());
        
    }

    public double getTrajectory(){
        results = LimelightHelpers.getLatestResults(TurretConstants.limelightName);
        Pose3d robotPos = results.getBotPose3d();
        distanceVector.x = robotPos.getX();
        distanceVector.y = robotPos.getZ();

        return distanceVector.mag(); //replace this with the function once gotten

    }

    public void checkForTuning(){
        boolean valueHasChanged = false; 

        if (TurretSettings.kP != lastkP){
            lastkP = TurretSettings.kP;
            flywheelConfig.Slot0.kP = lastkP;
            valueHasChanged = true;
        }

        if (TurretSettings.kI != lastkI){
            lastkI = TurretSettings.kI;
            flywheelConfig.Slot0.kI = lastkI;
            valueHasChanged = true;
        }

        if (TurretSettings.kD != lastkD){
            lastkD = TurretSettings.kD;
            flywheelConfig.Slot0.kD = lastkD;
            valueHasChanged = true;
        }

        if (TurretSettings.kS != lastkS){
            lastkS = TurretSettings.kS;
            flywheelConfig.Slot0.kS = lastkS;
            valueHasChanged = true;
        }

        if (TurretSettings.kV != lastkV){
            lastkV = TurretSettings.kV;
            flywheelConfig.Slot0.kV = lastkV;
            valueHasChanged = true;
        }

        if (TurretSettings.kA != lastkA){
            lastkA = TurretSettings.kA;
            flywheelConfig.Slot0.kA = lastkA;
            valueHasChanged = true;
        }

        if (valueHasChanged){
            leftFlyMotor.getConfigurator().apply(flywheelConfig);
            rightFlyMotor.getConfigurator().apply(flywheelConfig);
        }

    }

    public void tune(){
        function.value(getTrajectory());

        rightFlyMotor.setControl(new MotionMagicVelocityDutyCycle(TurretSettings.setVelocities)); 
        leftFlyMotor.setControl(new Follower(MotorConstants.rightFlyID, MotorAlignmentValue.Opposed)); 
        
    }


    @Override
    public void periodic(){

        TurretSettings.kP = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kP);
        TurretSettings.kI = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kI);
        TurretSettings.kD = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kD);
        TurretSettings.kS = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kS);
        TurretSettings.kV = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kV);
        TurretSettings.kA = SmartDashboard.getNumber("Flywheel kP", TurretSettings.kA);
        TurretSettings.setVelocities = SmartDashboard.getNumber("Flywheel Speeds", TurretSettings.setVelocities);

        
        checkForTuning();
    }
    
}
