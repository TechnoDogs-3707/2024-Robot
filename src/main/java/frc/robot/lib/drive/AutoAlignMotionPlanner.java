package frc.robot.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.motion.IMotionProfileGoal;
import frc.robot.lib.motion.MotionProfileGoal;
import frc.robot.lib.motion.MotionState;
import frc.robot.lib.motion.ProfileFollower;

import java.util.OptionalDouble;

public class AutoAlignMotionPlanner {

    private ProfileFollower mXController = new ProfileFollower(2.5, 0.0, 0.0, 1.0, 0.0, 0.0);
    private ProfileFollower mYController = new ProfileFollower(2.5, 0.0, 0.0, 1.0, 0.0, 0.0);
    private ProfileFollower mThetaController = new ProfileFollower(1.5, 0.0, 0.0, 1.0, 0.0, 0.0);

    boolean mAutoAlignComplete = false;

    private Pose2d mFieldToTargetPoint;
    private OptionalDouble mStartTime;

    public void reset() {
        mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
        mXController.resetProfile();
        mXController.resetSetpoint();
        mYController.resetProfile();
        mYController.resetSetpoint();
        mThetaController.resetProfile();
        mThetaController.resetSetpoint();
        mAutoAlignComplete = false;
    }

    public synchronized void setTargetPoint(Pose2d targetPoint) {
        mFieldToTargetPoint = targetPoint;

    }

    public synchronized ChassisSpeeds updateAutoAlign(double timestamp, Pose2d currentPose, Twist2d current_vel) {
        var odom_to_target_point = mFieldToTargetPoint;

        mXController.setGoalAndConstraints(
            new MotionProfileGoal(odom_to_target_point.getTranslation().getX(), 0, IMotionProfileGoal.CompletionBehavior.OVERSHOOT, 0.02, 0.05),
            Constants.kPositionMotionProfileConstraints);
        mYController.setGoalAndConstraints(
            new MotionProfileGoal(odom_to_target_point.getTranslation().getY(), 0, IMotionProfileGoal.CompletionBehavior.OVERSHOOT, 0.02, 0.05),
            Constants.kPositionMotionProfileConstraints);
        mThetaController.setGoalAndConstraints(
            new MotionProfileGoal(odom_to_target_point.getRotation().getRadians(), 0, IMotionProfileGoal.CompletionBehavior.OVERSHOOT, 0.03, 0.05),
            Constants.kHeadingMotionProfileConstraints);

        double currentRotation = currentPose.getRotation().getRadians();

        Translation2d current_vel_robot_frame = new Translation2d(current_vel.dx, current_vel.dy);
        Translation2d current_vel_odom_frame = current_vel_robot_frame.rotateBy(currentPose.getRotation());

        if (odom_to_target_point.getRotation().getRadians() - currentRotation > Math.PI) {
            currentRotation += 2 * Math.PI;
        } else if (odom_to_target_point.getRotation().getRadians() - currentRotation < -Math.PI) {
            currentRotation -= 2 * Math.PI;
        }

        double xOutput = mXController.update(
               new MotionState(timestamp, currentPose.getTranslation().getX(), current_vel_odom_frame.getX(), 0.0),
                timestamp + Constants.loopPeriodSecs);
        double yOutput = mYController.update(
               new MotionState(timestamp, currentPose.getTranslation().getY(), current_vel_odom_frame.getY(), 0.0),
                timestamp + Constants.loopPeriodSecs);
        double thetaOutput = mThetaController.update(
                new MotionState(timestamp, currentRotation, current_vel.dtheta, 0.0),
                timestamp + Constants.loopPeriodSecs);

        ChassisSpeeds setpoint;

        boolean thetaWithinDeadband = mThetaController.onTarget();
        boolean xWithinDeadband = mXController.onTarget();
        boolean yWithinDeadband = mYController.onTarget();

        setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                xWithinDeadband ? 0.0 : xOutput,
                yWithinDeadband ? 0.0 : yOutput,
                thetaWithinDeadband ? 0.0 : thetaOutput,
                currentPose.getRotation());
        mAutoAlignComplete = thetaWithinDeadband && xWithinDeadband && yWithinDeadband;

        if (mStartTime.isPresent() && mAutoAlignComplete) {
            System.out.println("Auto align took: " + (Timer.getFPGATimestamp() - mStartTime.getAsDouble()));
            mStartTime = OptionalDouble.empty();
        }

        return setpoint;
    }

    public synchronized boolean getAutoAlignComplete() {
        return mAutoAlignComplete;
    }
}
