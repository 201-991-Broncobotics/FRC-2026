package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Settings.RollerSettings;

public class RollerSubsystem extends SubsystemBase {

    private TalonFX intakeMotor, intakeMotor2, pivotMotor; 
    private TalonFXConfiguration intakeMotorConfig, intakeMotor2Config, pivotMotorConfig;
    private StatusCode intakeMotorStatus, intakeMotor2Status, pivotMotorStatus; 
    private double highTargetPosition, lastVelo, lastAcc, lastkP, lastkI, lastkD, lastkG; 
    //private CurrentLimitsConfigs currentLimits; 

    public RollerSubsystem(){

        highTargetPosition = (RollerConstants.highLimitAngle/(2.0 * Math.PI)) * RollerConstants.gearRatio; 

        intakeMotor = new TalonFX(MotorConstants.rollerIntakeID); 
        intakeMotor2 = new TalonFX(MotorConstants.rollerIntake2ID); 
        pivotMotor = new TalonFX(MotorConstants.pivotID); 

        intakeMotorConfig = new TalonFXConfiguration();
        intakeMotor2Config = new TalonFXConfiguration();
        pivotMotorConfig = new TalonFXConfiguration();

        intakeMotorConfig.Voltage.PeakForwardVoltage = RollerConstants.maxForwardVoltage; 
        intakeMotorConfig.Voltage.PeakReverseVoltage = RollerConstants.maxReverseVoltage; 
        intakeMotor2Config.Voltage.PeakForwardVoltage = RollerConstants.maxForwardVoltage; 
        intakeMotor2Config.Voltage.PeakReverseVoltage = RollerConstants.maxReverseVoltage; 
        pivotMotorConfig.Voltage.PeakForwardVoltage = RollerConstants.maxForwardVoltage; 
        pivotMotorConfig.Voltage.PeakReverseVoltage = RollerConstants.maxReverseVoltage; 

        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        intakeMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       
        //Set current if needed 
        /*currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true; 
        currentLimits.SupplyCurrentLimit = 40; 
        currentLimits.StatorCurrentLimitEnable = true; 
        currentLimits.StatorCurrentLimit = 60;
        
        intakeMotorConfig.CurrentLimits = currentLimits;
        intakeMotor2Config.CurrentLimits = currentLimits; */

        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = RollerSettings.pivotMotorVelocity; 
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = RollerSettings.pivotMotorAcceleration; 
        pivotMotorConfig.Slot0.kP = RollerSettings.pivotkP; 
        pivotMotorConfig.Slot0.kI = RollerSettings.pivotkI; 
        pivotMotorConfig.Slot0.kD = RollerSettings.pivotkD; 
        pivotMotorConfig.Slot0.kG = RollerSettings.pivotkG;
    

        intakeMotor.getConfigurator().apply(intakeMotorConfig); 
        intakeMotor2.getConfigurator().apply(intakeMotor2Config); 
        pivotMotor.getConfigurator().apply(pivotMotorConfig); 

        intakeMotorStatus = intakeMotor.getConfigurator().apply(intakeMotorConfig);
        intakeMotor2Status = intakeMotor2.getConfigurator().apply(intakeMotor2Config); 
        pivotMotorStatus = pivotMotor.getConfigurator().apply(pivotMotorConfig);

        lastVelo = RollerSettings.pivotMotorVelocity; 
        lastAcc = RollerSettings.pivotMotorAcceleration; 
        lastkP = RollerSettings.pivotkP; 
        lastkI = RollerSettings.pivotkI; 
        lastkP = RollerSettings.pivotkD; 
        lastkG = RollerSettings.pivotkG; 

        if (!intakeMotorStatus.isOK()) System.out.println("Intake Motor with ID " + MotorConstants.rollerIntakeID + " is broken!");
        if (!intakeMotor2Status.isOK()) System.out.println("Intake Motor with ID " + MotorConstants.rollerIntake2ID + " is broken!");
        if (!pivotMotorStatus.isOK()) System.out.println("Intake Motor with ID " + MotorConstants.pivotID + " is broken!");

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
        intakeMotor2.setVoltage(RollerSettings.runningVoltage);

    }

    public void stall(){
        intakeMotor.setVoltage(RollerSettings.defaultVoltage);
        intakeMotor2.setVoltage(RollerSettings.defaultVoltage);
    }

    public void stopRollers(){
        intakeMotor.stopMotor();
        intakeMotor2.stopMotor();
    }

    public void lift(){
        pivotMotor.setControl(new MotionMagicVoltage(highTargetPosition)); 

    }

    public void drop(){
        pivotMotor.setControl(new MotionMagicVoltage(RollerConstants.startingPosition)); 
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

        if (valueHasChanged) pivotMotor.getConfigurator().apply(pivotMotorConfig);
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
