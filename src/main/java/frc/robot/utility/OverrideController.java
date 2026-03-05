package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OverrideController extends CommandXboxController {
    
    private final CommandXboxController singlePlayer;
    private final CommandXboxController override;
    private double deadband;

    /**
     * @param dummyPort A port on the Driver Station that is EMPTY (e.g., 5)
     * @param singlePlayer The main driver controller
     * @param override The operator controller (has priority)
     * @param deadband Joystick deadband to prevent drift overrides
     */
    public OverrideController(int dummyPort, CommandXboxController singlePlayer, CommandXboxController override, double deadband) {
        super(dummyPort); // Tells WPILib this is technically a controller on the dummy port
        this.singlePlayer = singlePlayer;
        this.override = override;
        this.deadband = deadband;
    }

    public OverrideController(int dummyPort, CommandXboxController singlePlayer, CommandXboxController override) {
        this(dummyPort, singlePlayer, override, 0.05);
    }

    // ==========================================
    // AXIS OVERRIDES
    // ==========================================

    private double getPriorityAxis(double overrideVal, double singlePlayerVal) {
        if (Math.abs(overrideVal) > deadband) return overrideVal;
        if (Math.abs(singlePlayerVal) > deadband) return singlePlayerVal;
        return 0.0;
    }

    @Override
    public double getLeftY() { return getPriorityAxis(override.getLeftY(), singlePlayer.getLeftY()); }
    
    @Override
    public double getRightY() { return getPriorityAxis(override.getRightY(), singlePlayer.getRightY()); }
    
    @Override
    public double getLeftX() { return getPriorityAxis(override.getLeftX(), singlePlayer.getLeftX()); }
    
    @Override
    public double getRightX() { return getPriorityAxis(override.getRightX(), singlePlayer.getRightX()); }
    
    @Override
    public double getLeftTriggerAxis() { return getPriorityAxis(override.getLeftTriggerAxis(), singlePlayer.getLeftTriggerAxis()); }
    
    @Override
    public double getRightTriggerAxis() { return getPriorityAxis(override.getRightTriggerAxis(), singlePlayer.getRightTriggerAxis()); }

    public double getCombinedTriggerAxis() { return getRightTriggerAxis() - getLeftTriggerAxis();}

    // ==========================================
    // BUTTON OVERRIDES
    // ==========================================

    @Override
    public Trigger a() { return singlePlayer.a().or(override.a()); }
    
    @Override
    public Trigger b() { return singlePlayer.b().or(override.b()); }
    
    @Override
    public Trigger x() { return singlePlayer.x().or(override.x()); }
    
    @Override
    public Trigger y() { return singlePlayer.y().or(override.y()); }

    @Override
    public Trigger leftBumper() { return singlePlayer.leftBumper().or(override.leftBumper()); }
    
    @Override
    public Trigger rightBumper() { return singlePlayer.rightBumper().or(override.rightBumper()); }

    @Override
    public Trigger leftStick() { return singlePlayer.leftStick().or(override.leftStick()); }
    
    @Override
    public Trigger rightStick() { return singlePlayer.rightStick().or(override.rightStick()); }

    @Override
    public Trigger back() { return singlePlayer.back().or(override.back()); }
    
    @Override
    public Trigger start() { return singlePlayer.start().or(override.start()); }

    // ==========================================
    // D-PAD (POV) OVERRIDES
    // ==========================================

    @Override
    public Trigger pov(int angle) { return singlePlayer.pov(angle).or(override.pov(angle)); }
    
    @Override
    public Trigger povUp() { return singlePlayer.povUp().or(override.povUp()); }
    
    @Override
    public Trigger povDown() { return singlePlayer.povDown().or(override.povDown()); }
    
    @Override
    public Trigger povLeft() { return singlePlayer.povLeft().or(override.povLeft()); }
    
    @Override
    public Trigger povRight() { return singlePlayer.povRight().or(override.povRight()); }
}