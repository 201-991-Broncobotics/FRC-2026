package frc.robot.utility;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OverrideController extends CommandXboxController {
    private final CommandXboxController singlePlayer;
    private final CommandXboxController override;
    private final double deadband;

    /**
     * @param dummyPort    A port on the Driver Station that is EMPTY (e.g., 5)
     * @param singlePlayer The main driver controller
     * @param override     The operator controller (has priority)
     * @param deadband     Joystick deadband to prevent drift overrides
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
    // RAW HID OVERRIDE
    // ==========================================

    /**
     * Note: You cannot combine two raw XboxController objects. 
     * Defaulting to the singlePlayer's raw HID. 
     */
    @Override
    public XboxController getHID() {
        return singlePlayer.getHID();
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
    public double getLeftX() { return getPriorityAxis(override.getLeftX(), singlePlayer.getLeftX()); }
    @Override
    public double getRightX() { return getPriorityAxis(override.getRightX(), singlePlayer.getRightX()); }
    @Override
    public double getLeftY() { return getPriorityAxis(override.getLeftY(), singlePlayer.getLeftY()); }
    @Override
    public double getRightY() { return getPriorityAxis(override.getRightY(), singlePlayer.getRightY()); }
    @Override
    public double getLeftTriggerAxis() { return getPriorityAxis(override.getLeftTriggerAxis(), singlePlayer.getLeftTriggerAxis()); }
    @Override
    public double getRightTriggerAxis() { return getPriorityAxis(override.getRightTriggerAxis(), singlePlayer.getRightTriggerAxis()); }

    public double getCombinedTriggerAxis() { return getRightTriggerAxis() - getLeftTriggerAxis(); }

    // ==========================================
    // BUTTON OVERRIDES
    // ==========================================

    @Override
    public Trigger a() { return singlePlayer.a().or(override.a()); }
    @Override
    public Trigger a(EventLoop loop) { return singlePlayer.a(loop).or(override.a(loop)); }

    @Override
    public Trigger b() { return singlePlayer.b().or(override.b()); }
    @Override
    public Trigger b(EventLoop loop) { return singlePlayer.b(loop).or(override.b(loop)); }

    @Override
    public Trigger x() { return singlePlayer.x().or(override.x()); }
    @Override
    public Trigger x(EventLoop loop) { return singlePlayer.x(loop).or(override.x(loop)); }

    @Override
    public Trigger y() { return singlePlayer.y().or(override.y()); }
    @Override
    public Trigger y(EventLoop loop) { return singlePlayer.y(loop).or(override.y(loop)); }

    @Override
    public Trigger leftBumper() { return singlePlayer.leftBumper().or(override.leftBumper()); }
    @Override
    public Trigger leftBumper(EventLoop loop) { return singlePlayer.leftBumper(loop).or(override.leftBumper(loop)); }

    @Override
    public Trigger rightBumper() { return singlePlayer.rightBumper().or(override.rightBumper()); }
    @Override
    public Trigger rightBumper(EventLoop loop) { return singlePlayer.rightBumper(loop).or(override.rightBumper(loop)); }

    @Override
    public Trigger back() { return singlePlayer.back().or(override.back()); }
    @Override
    public Trigger back(EventLoop loop) { return singlePlayer.back(loop).or(override.back(loop)); }

    @Override
    public Trigger start() { return singlePlayer.start().or(override.start()); }
    @Override
    public Trigger start(EventLoop loop) { return singlePlayer.start(loop).or(override.start(loop)); }

    @Override
    public Trigger leftStick() { return singlePlayer.leftStick().or(override.leftStick()); }
    @Override
    public Trigger leftStick(EventLoop loop) { return singlePlayer.leftStick(loop).or(override.leftStick(loop)); }

    @Override
    public Trigger rightStick() { return singlePlayer.rightStick().or(override.rightStick()); }
    @Override
    public Trigger rightStick(EventLoop loop) { return singlePlayer.rightStick(loop).or(override.rightStick(loop)); }

    // ==========================================
    // TRIGGER AS BUTTON OVERRIDES
    // ==========================================

    @Override
    public Trigger leftTrigger(double threshold, EventLoop loop) { 
        return singlePlayer.leftTrigger(threshold, loop).or(override.leftTrigger(threshold, loop)); 
    }
    @Override
    public Trigger leftTrigger(double threshold) { 
        return singlePlayer.leftTrigger(threshold).or(override.leftTrigger(threshold)); 
    }
    @Override
    public Trigger leftTrigger() { 
        return singlePlayer.leftTrigger().or(override.leftTrigger()); 
    }

    @Override
    public Trigger rightTrigger(double threshold, EventLoop loop) { 
        return singlePlayer.rightTrigger(threshold, loop).or(override.rightTrigger(threshold, loop)); 
    }
    @Override
    public Trigger rightTrigger(double threshold) { 
        return singlePlayer.rightTrigger(threshold).or(override.rightTrigger(threshold)); 
    }
    @Override
    public Trigger rightTrigger() { 
        return singlePlayer.rightTrigger().or(override.rightTrigger()); 
    }

    // ==========================================
    // D-PAD (POV) OVERRIDES (Inherited from CommandGenericHID)
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