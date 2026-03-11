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
import frc.robot.Constants.TraverseConstants;
import frc.robot.Settings.TraverseSettings;
import frc.robot.utility.ElapsedTime;

public class TraverseSubsystem extends SubsystemBase {

    private TalonFX rollerMotor, scoopMotor; 
    private TalonFXConfiguration rollerMotorConfig, scoopMotorConfig; 
    private StatusCode rollerMotorStatus, scoopMotorStatus;
    private CurrentLimitsConfigs rollerCurrentLimits, scoopCurrentLimits; 

    public TraverseSubsystem(){

        rollerMotor = new TalonFX(MotorConstants.traverseRollerID);
        scoopMotor = new TalonFX(MotorConstants.traverseScoopID);

        rollerMotorConfig = new TalonFXConfiguration();
        scoopMotorConfig = new TalonFXConfiguration();

        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //rollerMotorConfig.Voltage.PeakForwardVoltage = TraverseConstants.maxForwardVoltage; 
        //rollerMotorConfig.Voltage.PeakReverseVoltage = TraverseConstants.maxReverseVoltage; 

        scoopMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        scoopMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //scoopMotorConfig.Voltage.PeakForwardVoltage = TraverseConstants.maxForwardVoltage; 
        //scoopMotorConfig.Voltage.PeakReverseVoltage = TraverseConstants.maxReverseVoltage; 

        //add Current Limits? 
        rollerCurrentLimits = new CurrentLimitsConfigs();
        rollerCurrentLimits.SupplyCurrentLimitEnable = TraverseConstants.currentLimitsEnabled; 
        rollerCurrentLimits.SupplyCurrentLimit = TraverseConstants.rollerSupplyCurrent; 
        rollerCurrentLimits.StatorCurrentLimitEnable = false; //TraverseConstants.currentLimitsEnabled; 
        rollerCurrentLimits.StatorCurrentLimit = TraverseConstants.rollerStatorCurrent; 
        rollerMotorConfig.CurrentLimits = rollerCurrentLimits; 

        scoopCurrentLimits = new CurrentLimitsConfigs();
        scoopCurrentLimits.SupplyCurrentLimitEnable = TraverseConstants.currentLimitsEnabled; 
        scoopCurrentLimits.SupplyCurrentLimit = TraverseConstants.rollerSupplyCurrent; 
        scoopCurrentLimits.StatorCurrentLimitEnable = false; //TraverseConstants.currentLimitsEnabled; 
        scoopCurrentLimits.StatorCurrentLimit = TraverseConstants.rollerStatorCurrent; 
        scoopMotorConfig.CurrentLimits = scoopCurrentLimits; 

        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        scoopMotor.getConfigurator().apply(scoopMotorConfig);

        rollerMotorStatus = rollerMotor.getConfigurator().apply(rollerMotorConfig);
        scoopMotorStatus = scoopMotor.getConfigurator().apply(scoopMotorConfig);

        if (!rollerMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Traverse roller motor with ID " + MotorConstants.traverseRollerID + " is broken!");
        if (!scoopMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Traverse scoop motor with ID " + MotorConstants.traverseScoopID + " is broken!");

    }

    public void transfer() { rollerMotor.set(-TraverseSettings.rollerMotorPower); }
    public void scoop() { scoopMotor.set(TraverseSettings.scoopMotorPower); }

    public void emergencyReverse(){ rollerMotor.set(TraverseSettings.rollerMotorPower); }
    public void emergencyReverseScoop(){ scoopMotor.set(-TraverseSettings.scoopMotorPower); }

    public void stopRoller(){ rollerMotor.stopMotor(); }
    public void stopScoop() { scoopMotor.stopMotor(); }

    @Override
    public void periodic(){

        try {
            //SmartDashboard.putNumber("Traverse Roller Motor Temperature", rollerMotor.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Traverse Roller Motor RPM", rollerMotor.getVelocity().getValueAsDouble() * 60);
            SmartDashboard.putNumber("Traverse Roller Motor Current", rollerMotor.getTorqueCurrent().getValueAsDouble());
            //SmartDashboard.putNumber("Traverse Scoop Motor Temperature", scoopMotor.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Traverse Scoop Motor RPM", scoopMotor.getVelocity().getValueAsDouble() * 60);
            SmartDashboard.putNumber("Traverse Scoop Motor Current", scoopMotor.getTorqueCurrent().getValueAsDouble());
        } catch (NullPointerException e) {
            // do nothing
        }
        
       

    }
    
}
