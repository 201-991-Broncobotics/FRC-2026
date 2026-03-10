package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DropIntakeCommand extends Command {
    private final IntakeSubsystem intake;

    public DropIntakeCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
        addRequirements(intake); // Declare subsystem requirements
    }

    @Override
    public void initialize() {
        intake.drop();
        intake.update();
    }

    @Override
    public void execute() {
        intake.update();
    }

    @Override
    public boolean isFinished() {
        return intake.getIntakePivotPosition() < IntakeConstants.outIntakePosition - Math.toRadians(10); // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
        intake.stopPivotCustom();
        intake.update();
    }
}
