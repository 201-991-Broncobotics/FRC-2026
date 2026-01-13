package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.StorageConstants;

public class StorageSubsystem extends SubsystemBase {

    private TalonFX transferMotor; 
    private SparkMax agitatorMotor; //MIGHT be a SparkFlex idk yet  
    private TalonFXConfiguration transferMotorConfig; 
    private SparkBaseConfig agitatorMotorConfig; 

    public StorageSubsystem(){

        transferMotor = new TalonFX(MotorConstants.transferID); 
        agitatorMotor = new SparkMax(MotorConstants.agitatorID, MotorType.kBrushless); 

        transferMotorConfig = new TalonFXConfiguration(); 

        transferMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        transferMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //cus backwards = towards outtake
        transferMotorConfig.Voltage.PeakForwardVoltage = StorageConstants.maxForwardVoltage; 
        transferMotorConfig.Voltage.PeakReverseVoltage = StorageConstants.maxReverseVoltage; 

        //add Current Limits? 

        transferMotor.getConfigurator().apply(transferMotorConfig); 

        agitatorMotorConfig = new SparkMaxConfig(); 

        agitatorMotorConfig.idleMode(IdleMode.kBrake);
        agitatorMotorConfig.smartCurrentLimit(StorageConstants.vortexCurrentLimit);

        agitatorMotor.configure(agitatorMotorConfig, ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters); 

    }

    @Override
    public void periodic(){

    }
    
}
