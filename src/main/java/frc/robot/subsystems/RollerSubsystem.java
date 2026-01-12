package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.RollerSettings;

public class RollerSubsystem extends SubsystemBase {

    private TalonFX intakeMotor; 
    private StatusCode intakeMotorConfig; 
    private CurrentLimitsConfigs currentLimitConfigs;

    public RollerSubsystem(){

        intakeMotor = new TalonFX(MotorConstants.rollerIntakeID); 

        //Scary stuff that I don't know how works 
        /*currentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of())
        .withStatorCurrentLimitEnable(...);

        intakeMotorConfig = intakeMotor.getConfigurator().apply(currentLimitConfigs);
        */

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.putNumber("Roller Intake startingVoltage", RollerSettings.startingVoltage);
      

    }

    public void store(){
        

    }

    public void stall(){
        
    }

    @Override
    public void periodic(){

        RollerSettings.startingVoltage = SmartDashboard.getNumber("Roller Intake startingVoltage", RollerSettings.startingVoltage); 

    }


    
}
