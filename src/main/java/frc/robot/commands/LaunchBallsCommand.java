package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TraverseSubsystem;

public class LaunchBallsCommand extends Command {
    private final IntakeSubsystem intake;
    private final TraverseSubsystem traverse;

    public LaunchBallsCommand(IntakeSubsystem intakeSubsystem, TraverseSubsystem traverseSubsystem) {
        intake = intakeSubsystem;
        traverse = traverseSubsystem;
        addRequirements(intake); // Declare subsystem requirements
        addRequirements(traverse);
    }

    @Override
    public void initialize() {
        intake.feed();
        intake.agitate();
        intake.drop();
        intake.update();
        traverse.transfer();
        traverse.scoop();
    }

    @Override
    public void execute() {
        intake.update();
    }

    @Override
    public boolean isFinished() {
        return false; // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
        intake.stopRollers();
        intake.stopAgitate();
        intake.update();
        traverse.stopRoller();
        traverse.stopScoop();
    }
}
