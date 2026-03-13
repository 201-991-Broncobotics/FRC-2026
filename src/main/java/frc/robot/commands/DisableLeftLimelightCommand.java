package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;


public class DisableLeftLimelightCommand extends Command {

    public DisableLeftLimelightCommand() {
    }

    @Override
    public void initialize() {
        Settings.useLLimelight = false;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true; // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
    }
}
