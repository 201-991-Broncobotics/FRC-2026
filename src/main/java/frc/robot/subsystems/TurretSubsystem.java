package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Settings.TurretSettings;

public class TurretSubsystem extends SubsystemBase {

    private TalonFX leftFlyMotor, rightFlyMotor, turntableMotor; 
    private TalonFXConfiguration leftFlyMotorConfig, rightFlyMotorConfig, turntableMotorConfig;

    public TurretSubsystem(){

        leftFlyMotor = new TalonFX(MotorConstants.leftFlyID);
        rightFlyMotor = new TalonFX(MotorConstants.rightFlyID); 
        turntableMotor = new TalonFX(MotorConstants.turntableID); 

        leftFlyMotorConfig = new TalonFXConfiguration(); 
        rightFlyMotorConfig = new TalonFXConfiguration(); 
        turntableMotorConfig = new TalonFXConfiguration(); 

        leftFlyMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        rightFlyMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        turntableMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

        leftFlyMotorConfig.Voltage.PeakForwardVoltage = TurretConstants.maxForwardVoltage;
        leftFlyMotorConfig.Voltage.PeakReverseVoltage = TurretConstants.maxReverseVoltage; 
        rightFlyMotorConfig.Voltage.PeakForwardVoltage = TurretConstants.maxForwardVoltage; 
        rightFlyMotorConfig.Voltage.PeakReverseVoltage = TurretConstants.maxReverseVoltage;

    }

    @Override
    public void periodic(){

        
    }
    
}
