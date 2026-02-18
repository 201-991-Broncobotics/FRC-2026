package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Settings.RollerSettings;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeMotor, rightPivotMotor, leftPivotMotor; 
    private TalonFXConfiguration intakeMotorConfig, pivotMotorConfig;
    private StatusCode intakeMotorStatus, pivotMotorStatus;
    private double highTargetPosition, lastVelo, lastAcc, lastkP, lastkI, lastkD, lastkG; 
    private CurrentLimitsConfigs currentLimits;
    //private CurrentLimitsConfigs currentLimits; 

    public IntakeSubsystem(){

        highTargetPosition = (RollerConstants.highLimitAngle/(2.0 * Math.PI))/(RollerConstants.gearRatio); 

        intakeMotor = new TalonFX(MotorConstants.intakeID); 
        rightPivotMotor = new TalonFX(MotorConstants.rightIntakePivotID); 
        leftPivotMotor = new TalonFX(MotorConstants.leftIntakePivotID); 

        intakeMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig = new TalonFXConfiguration();
 
        intakeMotorConfig.Voltage.PeakForwardVoltage = RollerConstants.maxForwardVoltage; 
        intakeMotorConfig.Voltage.PeakReverseVoltage = RollerConstants.maxReverseVoltage; 
        //Add Time Constant if absolutely needed???? 
        //pivotMotorConfig.Voltage.SupplyVoltageTimeConstant = RollerConstants.voltageTimeConstant;
        pivotMotorConfig.Voltage.PeakForwardVoltage = RollerConstants.maxForwardVoltage; 
        pivotMotorConfig.Voltage.PeakReverseVoltage = RollerConstants.maxReverseVoltage; 

        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
        //Set currents if needed 
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = RollerConstants.currentLimitsEnabled; 
        currentLimits.SupplyCurrentLimit = RollerConstants.supplyCurrent;
        currentLimits.StatorCurrentLimitEnable = RollerConstants.currentLimitsEnabled; 
        currentLimits.StatorCurrentLimit = RollerConstants.statorCurrent; 
        
        intakeMotorConfig.CurrentLimits = currentLimits;
        pivotMotorConfig.CurrentLimits = currentLimits;

        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = RollerSettings.pivotMotorVelocity; 
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = RollerSettings.pivotMotorAcceleration; 
        pivotMotorConfig.Slot0.kP = RollerSettings.pivotkP; 
        pivotMotorConfig.Slot0.kI = RollerSettings.pivotkI; 
        pivotMotorConfig.Slot0.kD = RollerSettings.pivotkD; 
        pivotMotorConfig.Slot0.kG = RollerSettings.pivotkG;
    

        intakeMotor.getConfigurator().apply(intakeMotorConfig); 
        rightPivotMotor.getConfigurator().apply(pivotMotorConfig); 
        leftPivotMotor.getConfigurator().apply(pivotMotorConfig);

        intakeMotorStatus = intakeMotor.getConfigurator().apply(intakeMotorConfig);
        pivotMotorStatus = rightPivotMotor.getConfigurator().apply(pivotMotorConfig);
        
        lastVelo = RollerSettings.pivotMotorVelocity; 
        lastAcc = RollerSettings.pivotMotorAcceleration; 
        lastkP = RollerSettings.pivotkP; 
        lastkI = RollerSettings.pivotkI; 
        lastkP = RollerSettings.pivotkD; 
        lastkG = RollerSettings.pivotkG; 

        if (!intakeMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Roller motor with ID " + MotorConstants.intakeID + " is broken!");
        if (!pivotMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Pivot motors are broken!");
   

        SmartDashboard.putNumber("Roller Intake Default Voltage", RollerSettings.defaultVoltage);
        SmartDashboard.putNumber("Roller Intake Running Voltage", RollerSettings.runningVoltage);
        SmartDashboard.putNumber("Pivot Cruise Velocity", RollerSettings.pivotMotorVelocity); 
        SmartDashboard.putNumber("Pivot Acceleration", RollerSettings.pivotMotorAcceleration); 
        SmartDashboard.putNumber("Pivot kP", RollerSettings.pivotkP);
        SmartDashboard.putNumber("Pivot kI", RollerSettings.pivotkI); 
        SmartDashboard.putNumber("Pivot kD", RollerSettings.pivotkD);  
        SmartDashboard.putNumber("Pivot kG", RollerSettings.pivotkG); 
      
    }

    public void feed(){
        intakeMotor.setVoltage(RollerSettings.runningVoltage); 
        

    }

    public void stall(){
        intakeMotor.setVoltage(RollerSettings.defaultVoltage);
    }

    public void stopRollers(){
        intakeMotor.stopMotor();
    }

    public void lift(){
        rightPivotMotor.setControl(new MotionMagicVoltage(highTargetPosition).withSlot(0)); 
        leftPivotMotor.setControl(new Follower(MotorConstants.rightIntakePivotID, MotorAlignmentValue.Opposed));

    }

    public void drop(){
        rightPivotMotor.setControl(new MotionMagicVoltage(RollerConstants.startingPosition).withSlot(0)); 
        leftPivotMotor.setControl(new Follower(MotorConstants.rightIntakePivotID, MotorAlignmentValue.Opposed));
    }

    private void checkForTuning(){ //Updates values to allow tuning while robot is enabled
        boolean valueHasChanged = false; 

        if (RollerSettings.pivotMotorVelocity != lastVelo){

            lastVelo = RollerSettings.pivotMotorVelocity; 
            pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = lastVelo; 
            valueHasChanged = true;
        }

        if (RollerSettings.pivotMotorAcceleration != lastAcc){

            lastAcc = RollerSettings.pivotMotorAcceleration; 
            pivotMotorConfig.MotionMagic.MotionMagicAcceleration = lastAcc;
            valueHasChanged = true; 
        }

        if (RollerSettings.pivotkP != lastkP){

            lastkP = RollerSettings.pivotkP; 
            pivotMotorConfig.Slot0.kP = lastkP; 
            valueHasChanged = true; 
        }

        if (RollerSettings.pivotkI != lastkI){

            lastkI = RollerSettings.pivotkI; 
            pivotMotorConfig.Slot0.kI = lastkI; 
            valueHasChanged = true;
        }

        if (RollerSettings.pivotkD != lastkD){

            lastkD = RollerSettings.pivotkD; 
            pivotMotorConfig.Slot0.kD = lastkD;
            valueHasChanged = true;
        }

        if (RollerSettings.pivotkG != lastkG){

            lastkG = RollerSettings.pivotkG; 
            pivotMotorConfig.Slot0.kG = RollerSettings.pivotkG; 
            valueHasChanged = true; 
        }

        if (valueHasChanged) rightPivotMotor.getConfigurator().apply(pivotMotorConfig);
    }

    @Override
    public void periodic(){

        RollerSettings.defaultVoltage = SmartDashboard.getNumber("Roller Intake Default Voltage", RollerSettings.defaultVoltage);
        RollerSettings.runningVoltage = SmartDashboard.getNumber("Roller Intake Running Voltage", RollerSettings.runningVoltage); 
        RollerSettings.pivotMotorVelocity = SmartDashboard.getNumber("Pivot Cruise Velocity", RollerSettings.pivotMotorVelocity); 
        RollerSettings.pivotMotorAcceleration = SmartDashboard.getNumber("Pivot Acceleration", RollerSettings.pivotMotorAcceleration); 
        RollerSettings.pivotkP = SmartDashboard.getNumber("Pivot kP", RollerSettings.pivotkP);
        RollerSettings.pivotkI = SmartDashboard.getNumber("Pivot kI", RollerSettings.pivotkI); 
        RollerSettings.pivotkD = SmartDashboard.getNumber("Pivot kD", RollerSettings.pivotkD);  
        RollerSettings.pivotkG = SmartDashboard.getNumber("Pivot kG", RollerSettings.pivotkG); 

        checkForTuning();

    }


    
}
