package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Override {
    private final CommandXboxController singlePlayer;
    private final CommandXboxController override;
    private double deadband; // Removed 'final' so it can be updated

    /**
     * @param singlePlayer The primary driver controller (fallback)
     * @param override The operator controller (has absolute priority)
     * @param deadband The joystick deadband to prevent drift from triggering the override
     */
    public Override(CommandXboxController singlePlayer, CommandXboxController override, double deadband) {
        this.singlePlayer = singlePlayer;
        this.override = override;
        this.deadband = deadband;
    }

    public Override(CommandXboxController singlePlayer, CommandXboxController override) {
        this(singlePlayer, override, 0.05);
    }

    // ==========================================
    // UTILITY METHODS
    // ==========================================

    /** Allows you to update the deadband dynamically (e.g., from SmartDashboard) */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    /** Returns the base singleplayer controller (useful for setting rumble) */
    public CommandXboxController getSinglePlayerController() {
        return singlePlayer;
    }

    /** Returns the base override controller (useful for setting rumble) */
    public CommandXboxController getOverrideController() {
        return override;
    }

    // ==========================================
    // AXIS PRIORITY LOGIC (Joysticks & Triggers)
    // ==========================================

    private double getPriorityAxis(double overrideVal, double singlePlayerVal) {
        if (Math.abs(overrideVal) > deadband) {
            return overrideVal;
        }
        if (Math.abs(singlePlayerVal) > deadband) {
            return singlePlayerVal;
        }
        return 0.0;
    }

    public double getLeftY() { return getPriorityAxis(override.getLeftY(), singlePlayer.getLeftY()); }
    public double getRightY() { return getPriorityAxis(override.getRightY(), singlePlayer.getRightY()); }
    public double getLeftX() { return getPriorityAxis(override.getLeftX(), singlePlayer.getLeftX()); }
    public double getRightX() { return getPriorityAxis(override.getRightX(), singlePlayer.getRightX()); }
    
    public double getLeftTriggerAxis() { return getPriorityAxis(override.getLeftTriggerAxis(), singlePlayer.getLeftTriggerAxis()); }
    public double getRightTriggerAxis() { return getPriorityAxis(override.getRightTriggerAxis(), singlePlayer.getRightTriggerAxis()); }

    /** Returns the raw POV angle. Override wins if pressed (not -1). */
    public int getPOV() {
        if (override.getHID().getPOV() != -1) {
            return override.getHID().getPOV();
        }
        return singlePlayer.getHID().getPOV();
    }

    // ==========================================
    // BUTTON COMBINATION LOGIC
    // ==========================================

    public Trigger a() { return singlePlayer.a().or(override.a()); }
    public Trigger b() { return singlePlayer.b().or(override.b()); }
    public Trigger x() { return singlePlayer.x().or(override.x()); }
    public Trigger y() { return singlePlayer.y().or(override.y()); }

    public Trigger leftBumper() { return singlePlayer.leftBumper().or(override.leftBumper()); }
    public Trigger rightBumper() { return singlePlayer.rightBumper().or(override.rightBumper()); }

    public Trigger leftStick() { return singlePlayer.leftStick().or(override.leftStick()); }
    public Trigger rightStick() { return singlePlayer.rightStick().or(override.rightStick()); }

    public Trigger back() { return singlePlayer.back().or(override.back()); }
    public Trigger start() { return singlePlayer.start().or(override.start()); }

    // Triggers treated as boolean buttons (activates when pressed past a WPILib default threshold)
    public Trigger leftTrigger() { return singlePlayer.leftTrigger().or(override.leftTrigger()); }
    public Trigger rightTrigger() { return singlePlayer.rightTrigger().or(override.rightTrigger()); }

    // ==========================================
    // D-PAD (POV) COMBINATION LOGIC
    // ==========================================

    public Trigger pov(int angle) { return singlePlayer.pov(angle).or(override.pov(angle)); }
    public Trigger povUp() { return singlePlayer.povUp().or(override.povUp()); }
    public Trigger povDown() { return singlePlayer.povDown().or(override.povDown()); }
    public Trigger povLeft() { return singlePlayer.povLeft().or(override.povLeft()); }
    public Trigger povRight() { return singlePlayer.povRight().or(override.povRight()); }
    public Trigger povUpLeft() { return singlePlayer.povUpLeft().or(override.povUpLeft()); }
    public Trigger povUpRight() { return singlePlayer.povUpRight().or(override.povUpRight()); }
    public Trigger povDownLeft() { return singlePlayer.povDownLeft().or(override.povDownLeft()); }
    public Trigger povDownRight() { return singlePlayer.povDownRight().or(override.povDownRight()); }
    public Trigger povCenter() { return singlePlayer.povCenter().or(override.povCenter()); }
}