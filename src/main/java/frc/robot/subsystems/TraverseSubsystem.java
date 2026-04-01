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

    private TalonFX rollerMotor, rollerMotor2, scoopMotor; 
    private TalonFXConfiguration rollerMotorConfig, scoopMotorConfig; 
    private StatusCode rollerMotorStatus, scoopMotorStatus;
    private CurrentLimitsConfigs rollerCurrentLimits, scoopCurrentLimits; 

    public boolean agitateTraverse = false;
    public static boolean intakeAgitating = false;
    public boolean runningTransfer = false;
    public boolean runningScoop = false;
    private boolean temporarilyReversed = false;
    private boolean loweredMaxCurrent = false;

    private ElapsedTime reverseTimer;

    public TraverseSubsystem(){

        reverseTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        rollerMotor = new TalonFX(MotorConstants.traverseRollerID);
        rollerMotor2 = new TalonFX(MotorConstants.traverseRoller2ID);
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
        rollerMotor2.getConfigurator().apply(rollerMotorConfig);
        scoopMotor.getConfigurator().apply(scoopMotorConfig);

        rollerMotorStatus = rollerMotor.getConfigurator().apply(rollerMotorConfig);
        scoopMotorStatus = scoopMotor.getConfigurator().apply(scoopMotorConfig);

        if (!rollerMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Traverse roller motor with ID " + MotorConstants.traverseRollerID + " is broken!");
        if (!scoopMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Traverse scoop motor with ID " + MotorConstants.traverseScoopID + " is broken!");

    }

    public void update() { 

        /* 
        if (agitateTraverse) {
            if (runningTransfer && intakeAgitating) {
                rollerMotor.set(TraverseSettings.rollerMotorPower); 
                rollerMotor2.set(-TraverseSettings.rollerMotorPower); 
            }
            else if (runningTransfer && !intakeAgitating) {
                rollerMotor.set(-TraverseSettings.rollerMotorPower); 
                rollerMotor2.set(TraverseSettings.rollerMotorPower); 
            }

            if (runningScoop && intakeAgitating) scoopMotor.set(-TraverseSettings.scoopMotorPower); 
            else if (runningScoop && !intakeAgitating) scoopMotor.set(TraverseSettings.scoopMotorPower); 
        }*/

        if (scoopMotor.getTorqueCurrent().getValueAsDouble() > 15 && reverseTimer.time() > 1) {
            reverseTimer.reset();
            temporarilyReversed = true;
        }
        if (reverseTimer.time() < 0.5 && temporarilyReversed) {
            if (runningTransfer) emergencyReverse();
            if (runningScoop) emergencyReverseScoop();
        } else if (temporarilyReversed) {
            temporarilyReversed = false;
            if (runningTransfer) transfer();
            if (runningScoop) scoop();
        }

    }

    public void enableAgitate() { agitateTraverse = true; }
    public void disableAgitate() { agitateTraverse = false; }

    public void transfer() { 
        runningTransfer = true;
        rollerMotor.set(-TraverseSettings.rollerMotorPower); 
        rollerMotor2.set(TraverseSettings.rollerMotorPower); 
    }

    public void scoop() { // only run scoop if the turret has a chance of getting it in
        if (!OuttakeSubsystem.TurretWillMiss) {
            runningScoop = true;
            scoopMotor.set(TraverseSettings.scoopMotorPower); 
        } else if (runningScoop) stopScoop();
        
    }

    public void emergencyReverse(){ 
        rollerMotor.set(TraverseSettings.rollerMotorPower); 
        rollerMotor2.set(-TraverseSettings.rollerMotorPower); 
    }
    public void emergencyReverseScoop(){ scoopMotor.set(-TraverseSettings.scoopMotorPower); }

    public void stopRoller(){ 
        runningTransfer = false;
        rollerMotor.stopMotor(); 
        rollerMotor2.stopMotor();
    }
    public void stopScoop() { 
        runningScoop = false;
        scoopMotor.stopMotor(); 
    }

    @Override
    public void periodic(){

        try {

            if (DrivingProfiles.currentDriveSupplyCurrentLimit < 70 && !loweredMaxCurrent) {
                rollerCurrentLimits.SupplyCurrentLimit = TraverseConstants.rollerSupplyCurrent / 2; 
                rollerMotorConfig.CurrentLimits = rollerCurrentLimits; 
                scoopCurrentLimits.SupplyCurrentLimit = TraverseConstants.rollerSupplyCurrent / 2; 
                scoopMotorConfig.CurrentLimits = scoopCurrentLimits; 

                rollerMotor.getConfigurator().apply(rollerMotorConfig);
                rollerMotor2.getConfigurator().apply(rollerMotorConfig);
                scoopMotor.getConfigurator().apply(scoopMotorConfig);

                loweredMaxCurrent = true;
            }

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
