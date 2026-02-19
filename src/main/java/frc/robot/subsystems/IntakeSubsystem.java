package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Settings.IntakeSettings;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeMotor, rightPivotMotor, leftPivotMotor; 
    private TalonFXConfiguration intakeMotorConfig, pivotMotorConfig;
    private StatusCode intakeMotorStatus, pivotMotorStatus;
    private double highTargetPosition, lastVelo, lastAcc, lastkP, lastkI, lastkD, lastkG; 
    private CurrentLimitsConfigs currentLimits;
    private DoubleSupplier CurrentPivotPosition;
    private double pivotOffset = 0;
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    //private CurrentLimitsConfigs currentLimits; 

    public IntakeSubsystem(){

        intakeMotor = new TalonFX(MotorConstants.intakeID); 
        rightPivotMotor = new TalonFX(MotorConstants.rightIntakePivotID); 
        leftPivotMotor = new TalonFX(MotorConstants.leftIntakePivotID); 

        intakeMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig = new TalonFXConfiguration();
 
        intakeMotorConfig.Voltage.PeakForwardVoltage = IntakeConstants.maxForwardVoltage; 
        intakeMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.maxReverseVoltage; 
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //Add Time Constant if absolutely needed???? 
        //pivotMotorConfig.Voltage.SupplyVoltageTimeConstant = IntakeConstants.voltageTimeConstant;
        pivotMotorConfig.Voltage.PeakForwardVoltage = IntakeConstants.maxForwardVoltage; 
        pivotMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.maxReverseVoltage; 

        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
        //Set currents if needed 
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = IntakeConstants.currentLimitsEnabled; 
        currentLimits.SupplyCurrentLimit = IntakeConstants.supplyCurrent;
        currentLimits.StatorCurrentLimitEnable = IntakeConstants.currentLimitsEnabled; 
        currentLimits.StatorCurrentLimit = IntakeConstants.statorCurrent; 
        
        intakeMotorConfig.CurrentLimits = currentLimits;
        pivotMotorConfig.CurrentLimits = currentLimits;

        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotMotorConfig.Feedback.SensorToMechanismRatio = 1/IntakeConstants.gearRatio;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeSettings.pivotMotorVelocity; 
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = IntakeSettings.pivotMotorAcceleration; 
        pivotMotorConfig.Slot0.kP = IntakeSettings.pivotkP; 
        pivotMotorConfig.Slot0.kI = IntakeSettings.pivotkI; 
        pivotMotorConfig.Slot0.kD = IntakeSettings.pivotkD; 
        //pivotMotorConfig.Slot0.kG = IntakeSettings.pivotkG;
    

        intakeMotor.getConfigurator().apply(intakeMotorConfig); 
        rightPivotMotor.getConfigurator().apply(pivotMotorConfig); 
        leftPivotMotor.getConfigurator().apply(pivotMotorConfig);

        intakeMotorStatus = intakeMotor.getConfigurator().apply(intakeMotorConfig);
        pivotMotorStatus = rightPivotMotor.getConfigurator().apply(pivotMotorConfig);
        
        lastVelo = IntakeSettings.pivotMotorVelocity; 
        lastAcc = IntakeSettings.pivotMotorAcceleration; 
        lastkP = IntakeSettings.pivotkP; 
        lastkI = IntakeSettings.pivotkI; 
        lastkP = IntakeSettings.pivotkD; 
        lastkG = IntakeSettings.pivotkG; 

        pivotOffset = -rightPivotMotor.getPosition().getValueAsDouble() + IntakeConstants.startingPosition / (2*Math.PI);
        CurrentPivotPosition = () -> (rightPivotMotor.getPosition().getValueAsDouble() + pivotOffset) * 2*Math.PI;

        if (!intakeMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Roller motor with ID " + MotorConstants.intakeID + " is broken!");
        if (!pivotMotorStatus.isOK()) SmartDashboard.putString(getSubsystem(), "Pivot motors are broken!");
   

        SmartDashboard.putNumber("Roller Intake Default Power", IntakeSettings.defaultPower);
        SmartDashboard.putNumber("Roller Intake Running Power", IntakeSettings.runningPower);
        SmartDashboard.putNumber("Pivot Cruise Velocity", IntakeSettings.pivotMotorVelocity); 
        SmartDashboard.putNumber("Pivot Acceleration", IntakeSettings.pivotMotorAcceleration); 
        SmartDashboard.putNumber("Pivot kP", IntakeSettings.pivotkP);
        SmartDashboard.putNumber("Pivot kI", IntakeSettings.pivotkI); 
        SmartDashboard.putNumber("Pivot kD", IntakeSettings.pivotkD);  
        SmartDashboard.putNumber("Pivot kG", IntakeSettings.pivotkG); 
      
    }

    public void feed(){
        intakeMotor.set(IntakeSettings.runningPower); 
    }

    public void stall(){
        intakeMotor.set(IntakeSettings.defaultPower);
    }

    public void stopRollers(){
        intakeMotor.stopMotor();
    }

    public void lift(){
        double feedforward = getGravityFeedForward();
        rightPivotMotor.setControl(m_request.withPosition(IntakeConstants.highLimitAngle / (2*Math.PI) + pivotOffset).withFeedForward(feedforward)); 
        leftPivotMotor.setControl(new Follower(MotorConstants.rightIntakePivotID, MotorAlignmentValue.Opposed));

    }

    public void drop(){
        double feedforward = getGravityFeedForward();
        rightPivotMotor.setControl(m_request.withPosition(IntakeConstants.lowLimitAngle / (2*Math.PI) + pivotOffset).withFeedForward(feedforward)); 
        //rightPivotMotor.setControl(new MotionMagicVoltage(highTargetPosition / (2*Math.PI) + pivotOffset).withSlot(0)); 
        leftPivotMotor.setControl(new Follower(MotorConstants.rightIntakePivotID, MotorAlignmentValue.Opposed));
    }

    private double getGravityFeedForward() {
        // Get current arm position in rotations and convert to radians
        double angleRotations = CurrentPivotPosition.getAsDouble();
        
        // The output is proportional to the cosine of the angle
        return Math.sin(angleRotations) * -IntakeSettings.pivotkG;
    }

    private void checkForTuning(){ //Updates values to allow tuning while robot is enabled
        boolean valueHasChanged = false; 

        if (IntakeSettings.pivotMotorVelocity != lastVelo){

            lastVelo = IntakeSettings.pivotMotorVelocity; 
            pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = lastVelo; 
            valueHasChanged = true;
        }

        if (IntakeSettings.pivotMotorAcceleration != lastAcc){

            lastAcc = IntakeSettings.pivotMotorAcceleration; 
            pivotMotorConfig.MotionMagic.MotionMagicAcceleration = lastAcc;
            valueHasChanged = true; 
        }

        if (IntakeSettings.pivotkP != lastkP){

            lastkP = IntakeSettings.pivotkP; 
            pivotMotorConfig.Slot0.kP = lastkP; 
            valueHasChanged = true; 
        }

        if (IntakeSettings.pivotkI != lastkI){

            lastkI = IntakeSettings.pivotkI; 
            pivotMotorConfig.Slot0.kI = lastkI; 
            valueHasChanged = true;
        }

        if (IntakeSettings.pivotkD != lastkD){

            lastkD = IntakeSettings.pivotkD; 
            pivotMotorConfig.Slot0.kD = lastkD;
            valueHasChanged = true;
        }

        if (IntakeSettings.pivotkG != lastkG){

            lastkG = IntakeSettings.pivotkG; 
            //pivotMotorConfig.Slot0.kG = IntakeSettings.pivotkG; 
            valueHasChanged = true; 
        }

        if (valueHasChanged) rightPivotMotor.getConfigurator().apply(pivotMotorConfig);
    }

    @Override
    public void periodic(){

        IntakeSettings.defaultPower = SmartDashboard.getNumber("Roller Intake Default Voltage", IntakeSettings.defaultPower);
        IntakeSettings.runningPower = SmartDashboard.getNumber("Roller Intake Running Voltage", IntakeSettings.runningPower); 
        IntakeSettings.pivotMotorVelocity = SmartDashboard.getNumber("Pivot Cruise Velocity", IntakeSettings.pivotMotorVelocity); 
        IntakeSettings.pivotMotorAcceleration = SmartDashboard.getNumber("Pivot Acceleration", IntakeSettings.pivotMotorAcceleration); 
        IntakeSettings.pivotkP = SmartDashboard.getNumber("Pivot kP", IntakeSettings.pivotkP);
        IntakeSettings.pivotkI = SmartDashboard.getNumber("Pivot kI", IntakeSettings.pivotkI); 
        IntakeSettings.pivotkD = SmartDashboard.getNumber("Pivot kD", IntakeSettings.pivotkD);  
        IntakeSettings.pivotkG = SmartDashboard.getNumber("Pivot kG", IntakeSettings.pivotkG); 

        try {
            //if (CurrentPivotPosition.getAsDouble() < 0) pivotOffset = -rightPivotMotor.getPosition().getValueAsDouble();
            //if (CurrentPivotPosition.getAsDouble() > IntakeConstants.highLimitAngle) pivotOffset = -rightPivotMotor.getPosition().getValueAsDouble() + IntakeConstants.startingPosition / (2*Math.PI) / IntakeConstants.gearRatio;
            SmartDashboard.putNumber("Intake Pivot Position", Math.toDegrees(CurrentPivotPosition.getAsDouble()));
            SmartDashboard.putNumber("Intake Pivot Actual Position", rightPivotMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Intake Pivot Power", rightPivotMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Intake Pivot low target", IntakeConstants.lowLimitAngle / (2*Math.PI) + pivotOffset);

        } catch (NullPointerException e) {
            // do nothing
        }
        

        checkForTuning();

    }


    
}
