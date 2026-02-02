package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.Settings.StorageSettings;

public class StorageSubsystem extends SubsystemBase {

    private TalonFX traverseMotor; 
    private TalonFXConfiguration traverseMotorConfig; 
    private StatusCode traverseMotorStatus;
    private CurrentLimitsConfigs currentLimits; 

    public StorageSubsystem(){

        traverseMotor = new TalonFX(MotorConstants.traverseMotor);

        traverseMotorConfig = new TalonFXConfiguration();

        traverseMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        traverseMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        traverseMotorConfig.Voltage.PeakForwardVoltage = StorageConstants.maxForwardVoltage; 
        traverseMotorConfig.Voltage.PeakReverseVoltage = StorageConstants.maxReverseVoltage; 

        //add Current Limits? 
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = StorageConstants.currentLimitsEnabled; 
        currentLimits.SupplyCurrentLimit = StorageConstants.supplyCurrent; 
        currentLimits.StatorCurrentLimitEnable = StorageConstants.currentLimitsEnabled; 
        currentLimits.StatorCurrentLimit = StorageConstants.statorCurrent; 
        traverseMotorConfig.CurrentLimits = currentLimits; 

        traverseMotor.getConfigurator().apply(traverseMotorConfig);

        traverseMotorStatus = traverseMotor.getConfigurator().apply(traverseMotorConfig);

        if (!traverseMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Traverse motor with ID " + MotorConstants.traverseMotor + " is broken!");

        SmartDashboard.putNumber("Running Traverse Voltage", StorageSettings.runningTraverseMotor);

    }

    public void transfer(){

        traverseMotor.setVoltage(StorageSettings.runningTraverseMotor);

    }

    public void emergencyReverse(){

        traverseMotor.setVoltage(-StorageSettings.runningTraverseMotor);
    }

    public void stop(){

        traverseMotor.stopMotor();
        
    }

    @Override
    public void periodic(){

        StorageSettings.runningTraverseMotor =  SmartDashboard.getNumber("Running Traverse Voltage", StorageSettings.runningTraverseMotor);
       

    }
    
}
