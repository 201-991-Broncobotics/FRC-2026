package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


public class ResetElevatorCommand extends Command {
    private final ClimbingSubsystem climb;

    public ResetElevatorCommand(ClimbingSubsystem climbingSubsystem) {
        climb = climbingSubsystem;
        addRequirements(climb); // Declare subsystem requirements
    }

    @Override
    public void initialize() {
        climb.overrideRetract();
    }

    @Override
    public void execute() {
        climb.overrideRetract();
    }

    @Override
    public boolean isFinished() {
        return climb.getIfReachedResetPosition(); // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
        climb.stop();
    }
}
