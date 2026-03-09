package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;


public class EnableTurretCommand extends Command {
    private final OuttakeSubsystem outtake;

    public EnableTurretCommand(OuttakeSubsystem outtakeSystem) {
        outtake = outtakeSystem;
        addRequirements(outtake); // Declare subsystem requirements
    }

    @Override
    public void initialize() {
        outtake.startShooting();
        outtake.update();
    }

    @Override
    public void execute() {
        outtake.update();
    }

    @Override
    public boolean isFinished() {
        return false; // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
        outtake.stopShooting();
        outtake.update();
    }
}
