package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.drive.SwerveHeadingController;
import frc.robot.lib.drive.SwerveHeadingController.HeadingControllerState;
import frc.robot.subsystems.drive.Drive;

public class AutonDriveAimCommand extends Command {
    private final Drive mDrive;
    private final SwerveHeadingController mHeadingController;

    public AutonDriveAimCommand(Drive drive) {
        addRequirements(drive);
        mDrive = drive;
        mHeadingController = SwerveHeadingController.getInstance();
    }

    @Override
    public void initialize() {
        mHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
    }

    @Override
    public void execute() {
        final double mHeadingGoal = RobotState.getInstance().getAimingParameters().driveHeading().getDegrees();
        mHeadingController.setGoal(mHeadingGoal);

        // if (mHeadingController.getAbsError() <= Constants.kSwerveHeadingControllerMaintainThreshold) {
        //     mHeadingController.setHeadingControllerState(HeadingControllerState.MAINTAIN);
        // } else {
            mHeadingController.setHeadingControllerState(HeadingControllerState.SNAP);
        // }

        double omega = mHeadingController.update(RobotState.getInstance().getEstimatedPose().getRotation().getDegrees());

        mDrive.setVelocityClosedLoop(new ChassisSpeeds(0, 0, omega));
    }

    @Override
    public boolean isFinished() {
        //TODO: tune these to trade alignment time for accuracy
        return Math.abs(RobotState.getInstance().fieldVelocity().dtheta) < 0.5 && mHeadingController.getAbsError() < 3.7;
    }

    @Override
    public void end(boolean interrupted) {
        mHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
        mDrive.stop();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
