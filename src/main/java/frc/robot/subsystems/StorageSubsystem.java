package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.Settings.StorageSettings;

public class StorageSubsystem extends SubsystemBase {

    private TalonFX transferMotor, agitatorMotor;
    private TalonFXConfiguration transferMotorConfig, agitatorMotorConfig; 
    private StatusCode transferMotorStatus, agitatorMotorStatus; 

    public StorageSubsystem(){

        transferMotor = new TalonFX(MotorConstants.transferID); 
        agitatorMotor = new TalonFX(MotorConstants.agitatorID);

        transferMotorConfig = new TalonFXConfiguration(); 
        agitatorMotorConfig = new TalonFXConfiguration();

        transferMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        transferMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //cus backwards = towards outtake
        transferMotorConfig.Voltage.PeakForwardVoltage = StorageConstants.maxForwardVoltage; 
        transferMotorConfig.Voltage.PeakReverseVoltage = StorageConstants.maxReverseVoltage; 

        agitatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        agitatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        agitatorMotorConfig.Voltage.PeakForwardVoltage = StorageConstants.maxForwardVoltage; 
        agitatorMotorConfig.Voltage.PeakReverseVoltage = StorageConstants.maxReverseVoltage; 

        //add Current Limits? 

        transferMotor.getConfigurator().apply(transferMotorConfig); 
        agitatorMotor.getConfigurator().apply(agitatorMotorConfig);

        //agitatorMotorConfig.idleMode(IdleMode.kCoast);
        //agitatorMotorConfig.smartCurrentLimit(StorageConstants.vortexCurrentLimit);

        //agitatorMotor.configure(agitatorMotorConfig, ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters); 

        transferMotorStatus = transferMotor.getConfigurator().apply(transferMotorConfig);
        agitatorMotorStatus = agitatorMotor.getConfigurator().apply(agitatorMotorConfig);

        if (!transferMotorStatus.isOK()) System.out.println("Transfer motor with ID " + MotorConstants.transferID + " is broken!");
        if (!agitatorMotorStatus.isOK()) System.out.println("Agitator motor with ID " + MotorConstants.agitatorID + " is broken!");

        SmartDashboard.putNumber("Running Transfer Voltage", StorageSettings.runningTransferVoltage); 
        SmartDashboard.putNumber("Running Agitator Voltage", StorageSettings.runningAgitatorVoltage); 

    }

    public void agitate(){

        agitatorMotor.setVoltage(StorageSettings.runningAgitatorVoltage);

    }

    public void transfer(){

        transferMotor.setVoltage(StorageSettings.runningTransferVoltage);

    }

    public void stop(){

        agitatorMotor.stopMotor();
        transferMotor.stopMotor();
        
    }

    @Override
    public void periodic(){

        StorageSettings.runningAgitatorVoltage = SmartDashboard.getNumber("Running Agitator Voltage", StorageSettings.runningAgitatorVoltage); 
        StorageSettings.runningTransferVoltage = SmartDashboard.getNumber("Running Transfer Voltage", StorageSettings.runningTransferVoltage);

    }
    
}
