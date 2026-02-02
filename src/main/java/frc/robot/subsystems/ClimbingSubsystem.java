package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.ClimbingSettings;

public class ClimbingSubsystem extends SubsystemBase {

    private TalonFX climberMotor, elevatorMotor; 
    private TalonFXConfiguration climberMotorConfig, elevatorMotorConfig; 
    private StatusCode climberMotorStatus, elevatorMotorStatus; 
    private double lastCVelo, lastCAcc, lastCkP, lastCkI, lastCkD, lastCkG,
                    lastEVelo, lastEAcc, lastEkP, lastEkI, lastEkD, lastEkG,
                    climberRotations, elevatorRotations;  
    private CurrentLimitsConfigs currentLimits; 

    public ClimbingSubsystem(){

        
        climberRotations = ((ClimbingSettings.climberDistance)/(2 * Math.PI * ClimbingConstants.x60ShaftRadius)) * ClimbingConstants.gearRatio;
        elevatorRotations = ((ClimbingSettings.elevatorDistance)/(2 * Math.PI * ClimbingConstants.x60ShaftRadius)) * ClimbingConstants.gearRatio;

        climberMotor = new TalonFX(MotorConstants.climberID);
        elevatorMotor = new TalonFX(MotorConstants.elevatorID); 

        climberMotorConfig = new TalonFXConfiguration(); 
        elevatorMotorConfig = new TalonFXConfiguration();

        climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        climberMotorConfig.Voltage.PeakForwardVoltage = ClimbingConstants.maxForwardVoltage; 
        climberMotorConfig.Voltage.PeakReverseVoltage = ClimbingConstants.maxReverseVoltage; 
        climberMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        climberMotorConfig.Slot0.kG = ClimbingSettings.climberkG; 
        climberMotorConfig.Slot0.kP = ClimbingSettings.climberkP;
        climberMotorConfig.Slot0.kI = ClimbingSettings.climberkI; 
        climberMotorConfig.Slot0.kD = ClimbingSettings.climberkD; 
        climberMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbingSettings.climberMotorVelocity; 
        climberMotorConfig.MotionMagic.MotionMagicAcceleration = ClimbingSettings.climberMotorAcceleration; 

        //add current limits
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = ClimbingConstants.currentLimitsEnabled;
        currentLimits.SupplyCurrentLimit = ClimbingConstants.supplyCurrent;
        currentLimits.StatorCurrentLimitEnable = ClimbingConstants.currentLimitsEnabled;
        currentLimits.StatorCurrentLimit = ClimbingConstants.statorCurrent; 
        
        climberMotorConfig.CurrentLimits = currentLimits; 

        lastCkG = ClimbingSettings.climberkG; 
        lastCkP = ClimbingSettings.climberkP; 
        lastCkI = ClimbingSettings.climberkI; 
        lastCkD = ClimbingSettings.climberkD; 
        lastCVelo = ClimbingSettings.climberMotorVelocity; 
        lastCAcc = ClimbingSettings.climberMotorAcceleration; 
        
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        elevatorMotorConfig.Voltage.PeakForwardVoltage = ClimbingConstants.maxForwardVoltage; 
        elevatorMotorConfig.Voltage.PeakReverseVoltage = ClimbingConstants.maxReverseVoltage; 
        elevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
        elevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotorConfig.Slot0.kG = ClimbingSettings.elevatorkG; 
        elevatorMotorConfig.Slot0.kP = ClimbingSettings.elevatorkP; 
        elevatorMotorConfig.Slot0.kI = ClimbingSettings.elevatorkI; 
        elevatorMotorConfig.Slot0.kD = ClimbingSettings.elevatorkD; 
        elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbingSettings.elevatorMotorVelocity; 
        elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = ClimbingSettings.elevatorMotorAcceleration; 

        //add current limits
        elevatorMotorConfig.CurrentLimits = currentLimits;

        lastEkG = ClimbingSettings.elevatorkG; 
        lastEkP = ClimbingSettings.elevatorkP; 
        lastEkI = ClimbingSettings.elevatorkI; 
        lastEkD = ClimbingSettings.elevatorkD; 
        lastEVelo = ClimbingSettings.elevatorMotorVelocity; 
        lastEAcc = ClimbingSettings.elevatorMotorAcceleration;

        climberMotor.getConfigurator().apply(climberMotorConfig);
        elevatorMotor.getConfigurator().apply(elevatorMotorConfig); 

        climberMotorStatus = climberMotor.getConfigurator().apply(climberMotorConfig);
        elevatorMotorStatus = elevatorMotor.getConfigurator().apply(elevatorMotorConfig); 

        if (!climberMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Climbing motor with ID " + MotorConstants.climberID + " is broken!");
        if (!elevatorMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Elevator motor with ID " + MotorConstants.elevatorID + " is broken!");

        SmartDashboard.putNumber("Climber Cruise Velocity", ClimbingSettings.climberMotorVelocity); 
        SmartDashboard.putNumber("Climber Acceleration", ClimbingSettings.climberMotorAcceleration);
        SmartDashboard.putNumber("Climber kP", ClimbingSettings.climberkP); 
        SmartDashboard.putNumber("Climber kI", ClimbingSettings.climberkI); 
        SmartDashboard.putNumber("Climber kD", ClimbingSettings.climberkD); 
        SmartDashboard.putNumber("Climber kG", ClimbingSettings.climberkG); 

        SmartDashboard.putNumber("Elevator Cruise Velocity", ClimbingSettings.elevatorMotorVelocity); 
        SmartDashboard.putNumber("Elevator Acceleration", ClimbingSettings.elevatorMotorAcceleration); 
        SmartDashboard.putNumber("Elevator kP", ClimbingSettings.elevatorkP); 
        SmartDashboard.putNumber("Elevator kI", ClimbingSettings.elevatorkI); 
        SmartDashboard.putNumber("Elevator kD", ClimbingSettings.elevatorkD); 
        SmartDashboard.putNumber("Elevator kG", ClimbingSettings.elevatorkG); 

    }

    public void extend(){

        climberMotor.setControl(new MotionMagicVoltage(climberRotations).withSlot(0));
        elevatorMotor.setControl(new MotionMagicVoltage(elevatorRotations).withSlot(0)); 

    }

    public void retract(){

        climberMotor.setControl(new MotionMagicVoltage(ClimbingSettings.startingDistance).withSlot(0));
        elevatorMotor.setControl(new MotionMagicVoltage(ClimbingSettings.startingDistance).withSlot(0));  

    }

    

    private void checkForTuning(){ //Updates values to allow tuning while robot is enabled
        boolean climberValueHasChanged = false; 
        boolean elevatorValueHasChanged = false; 

        if (ClimbingSettings.climberMotorVelocity != lastCVelo){

            lastCVelo = ClimbingSettings.climberMotorVelocity;  
            climberMotorConfig.MotionMagic.MotionMagicCruiseVelocity = lastCVelo; 
            climberValueHasChanged = true;
        }

        if (ClimbingSettings.climberMotorAcceleration != lastCAcc){

            lastCAcc = ClimbingSettings.climberMotorAcceleration;  
            climberMotorConfig.MotionMagic.MotionMagicAcceleration = lastCAcc; 
            climberValueHasChanged = true;
        }

        if (ClimbingSettings.climberkP != lastCkP){

            lastCkP = ClimbingSettings.climberkP;  
            climberMotorConfig.Slot0.kP = lastCkP; 
            climberValueHasChanged = true;
        }

        if (ClimbingSettings.climberkI != lastCkI){

            lastCkI = ClimbingSettings.climberkI;  
            climberMotorConfig.Slot0.kI = lastCkI;  
            climberValueHasChanged = true;
        }

        if (ClimbingSettings.climberkD != lastCkD){

            lastCkD = ClimbingSettings.climberkD;  
            climberMotorConfig.Slot0.kD = lastCkD;  
            climberValueHasChanged = true;
        }

        if (ClimbingSettings.climberkG != lastCkG){

            lastCkG = ClimbingSettings.climberkG;  
            climberMotorConfig.Slot0.kG = lastCkG;  
            climberValueHasChanged = true;
        }

        if (ClimbingSettings.elevatorMotorVelocity != lastEVelo){

            lastEVelo = ClimbingSettings.elevatorMotorVelocity;  
            elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = lastEVelo; 
            elevatorValueHasChanged = true;
        }

        if (ClimbingSettings.elevatorMotorAcceleration != lastEAcc){

            lastEAcc = ClimbingSettings.elevatorMotorAcceleration;  
            elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = lastEAcc; 
            elevatorValueHasChanged = true;
        }

        if (ClimbingSettings.elevatorkP != lastEkP){

            lastEkP = ClimbingSettings.elevatorkP;  
            elevatorMotorConfig.Slot0.kP = lastEkP; 
            elevatorValueHasChanged = true;
        }

        if (ClimbingSettings.elevatorkI != lastEkI){

            lastEkI = ClimbingSettings.elevatorkI;  
            elevatorMotorConfig.Slot0.kI = lastEkI; 
            elevatorValueHasChanged = true;
        }

        if (ClimbingSettings.elevatorkD != lastEkD){

            lastEkD = ClimbingSettings.elevatorkD;  
            elevatorMotorConfig.Slot0.kD = lastEkD; 
            elevatorValueHasChanged = true;
        }

        if (ClimbingSettings.elevatorkG != lastEkG){

            lastEkG = ClimbingSettings.elevatorkG;  
            elevatorMotorConfig.Slot0.kG = lastEkG; 
            elevatorValueHasChanged = true;
        }

        if (climberValueHasChanged) climberMotor.getConfigurator().apply(climberMotorConfig);
        if (elevatorValueHasChanged) elevatorMotor.getConfigurator().apply(elevatorMotorConfig); 
    }



    @Override
    public void periodic(){

        ClimbingSettings.climberMotorVelocity = SmartDashboard.getNumber("Climber Cruise Velocity", ClimbingSettings.climberMotorVelocity); 
        ClimbingSettings.climberMotorAcceleration = SmartDashboard.getNumber("Climber Acceleration", ClimbingSettings.climberMotorAcceleration);
        ClimbingSettings.climberkP = SmartDashboard.getNumber("Climber kP", ClimbingSettings.climberkP); 
        ClimbingSettings.climberkI = SmartDashboard.getNumber("Climber kI", ClimbingSettings.climberkI); 
        ClimbingSettings.climberkD = SmartDashboard.getNumber("Climber kD", ClimbingSettings.climberkD); 
        ClimbingSettings.climberkG = SmartDashboard.getNumber("Climber kG", ClimbingSettings.climberkG); 

        ClimbingSettings.elevatorMotorVelocity = SmartDashboard.getNumber("Elevator Cruise Velocity", ClimbingSettings.elevatorMotorVelocity); 
        ClimbingSettings.elevatorMotorAcceleration = SmartDashboard.getNumber("Elevator Acceleration", ClimbingSettings.elevatorMotorAcceleration); 
        ClimbingSettings.elevatorkP = SmartDashboard.getNumber("Elevator kP", ClimbingSettings.elevatorkP); 
        ClimbingSettings.elevatorkI = SmartDashboard.getNumber("Elevator kI", ClimbingSettings.elevatorkI); 
        ClimbingSettings.elevatorkD = SmartDashboard.getNumber("Elevator kD", ClimbingSettings.elevatorkD); 
        ClimbingSettings.elevatorkG = SmartDashboard.getNumber("Elevator kG", ClimbingSettings.elevatorkG); 

        checkForTuning();
    
    }
    
}
