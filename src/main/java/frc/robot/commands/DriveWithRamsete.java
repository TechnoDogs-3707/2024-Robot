package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveWithRamsete extends Command {
    private final PIDController pidX, pidY;
    private final Constraints pidTConstraints;
    private final ProfiledPIDController pidT;
    private final HolonomicDriveController controller;
    private final Trajectory trajectory;

    private final Drive drive;

    private final Timer timer;

    public DriveWithRamsete(Drive d, Trajectory t) {
        pidX = new PIDController(1.5, 0, 0);
        pidY = new PIDController(1.5, 0, 0);
        pidTConstraints = new Constraints(2, 5);
        pidT = new ProfiledPIDController(1.5, 0, 0, pidTConstraints);
        controller = new HolonomicDriveController(pidY, pidX, pidT);
        
        trajectory = t;
        drive = d;
        timer = new Timer();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        var pose = drive.getPose();
        var time = timer.get();
        var driveTarget = trajectory.sample(time);
        var rotateTarget = drive.getAutonRotationTarget();
        var output = controller.calculate(pose, driveTarget, rotateTarget);
        drive.setVelocityClosedLoop(output);
    }
}
