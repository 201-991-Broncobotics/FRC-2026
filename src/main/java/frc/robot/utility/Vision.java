package frc.robot.utility;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.*;


public class Vision {
    private final NetworkTable m_limelightTable;
    private double tv, tx, ta;
    private ArrayList<Double> m_targetList;
    private final int MAX_ENTRIES = 50;
    // private final NetworkTableEntry m_isTargetValid, m_led_entry;


    /**
     * Creates a new Vision.
     */
    public Vision() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_targetList = new ArrayList<Double>(MAX_ENTRIES);
        // m_isTargetValid = ShuffleboardInfo.getInstance().getTargetEntry();
        // m_led_entry = m_limelightTable.getEntry("ledMode");

        m_limelightTable.getEntry("camMode").setNumber(0);
        m_limelightTable.getEntry("pipeline").setNumber(0);

    }

    public void updateVision() {
        // This method will be called once per scheduler run
        tv = m_limelightTable.getEntry("tv").getDouble(0);
        tx = m_limelightTable.getEntry("tx").getDouble(0);
        ta = m_limelightTable.getEntry("ta").getDouble(0);

        //m_isTargetValid.forceSetBoolean(isTargetValid());

        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
        m_targetList.add(ta);
    }

    public double getTX() {
        return tx;
    }

    public double getTA() {
        double sum = 0;

        for (Double num : m_targetList) { 		      
            sum += num.doubleValue();
        }
        return sum/m_targetList.size();
    }

    public boolean isTargetValid() {
        return (tv == 1.0); 
    }

}