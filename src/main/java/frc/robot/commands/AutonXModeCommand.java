package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutonXModeCommand extends Command {
    private final Drive drive;
    public AutonXModeCommand(Drive drive) {
        this.drive = drive;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        drive.stop();
        drive.setXMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
