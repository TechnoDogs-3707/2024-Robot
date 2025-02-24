package frc.robot.lib.drive;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.lib.motion.MotionProfileGoal;
import frc.robot.lib.motion.MotionState;
import frc.robot.lib.motion.ProfileFollower;
import frc.robot.util.poofsUtils.PoofsUtil;
import frc.robot.util.poofsUtils.TimeDelayedBoolean;

public class HeadingControlPlanner {
    private Optional<MotionProfileGoal> yaw_goal_ = Optional.empty();
    private TimeDelayedBoolean zero_yaw_rate_detector_ = new TimeDelayedBoolean();
    private final ProfileFollower profile_follower_ = new ProfileFollower(Constants.kHeadingControllerKp, Constants.kHeadingControllerKi, Constants.kHeadingControllerKd, Constants.kHeadingControllerKffv, Constants.kHeadingControllerKffa, Constants.kHeadingControllerKs);

    /**
     * Update chassis speeds with the rotation output from the PID profiler. Pass on translation values unchanged. When the yaw goal has been cleared, cache a new one
     *
     * @param desired
     * @return
     */
    public ChassisSpeeds update(double timestamp, ChassisSpeeds desired, Twist2d current_vel, Rotation2d gyro) {
        double current_rotation = gyro.getRadians();
        boolean yaw_rate_near_zero = zero_yaw_rate_detector_.update((Math.abs(current_vel.dtheta) < 0.03), .15);
        boolean has_translation_command = Math.hypot(desired.vxMetersPerSecond, desired.vyMetersPerSecond) > 0.1;

        if (yaw_goal_.isEmpty() && yaw_rate_near_zero) {
            yaw_goal_ = Optional.of(new MotionProfileGoal(current_rotation));
        }


        if (yaw_goal_.isPresent() && has_translation_command) {
            double current_error = yaw_goal_.get().pos() - current_rotation;
            if (current_error > Math.PI) {
                current_rotation += (2 * Math.PI);
            } else if (current_error < -Math.PI) {
                current_rotation -= (2 * Math.PI);
            }

            profile_follower_.setGoalAndConstraints(yaw_goal_.get(), Constants.kHeadingMotionProfileConstraints);
            double new_yaw_rate = profile_follower_.update(new MotionState(timestamp, current_rotation, current_vel.dtheta, /*acc=*/0.0), timestamp + Constants.loopPeriodSecs);

            // Return a new ChassisSpeeds that injects the controlled yaw rate
            return new ChassisSpeeds(desired.vxMetersPerSecond, desired.vyMetersPerSecond, new_yaw_rate);
        } else {
            return desired;
        }
    }

    public void reset() {
        yaw_goal_ = Optional.empty();
        profile_follower_.resetProfile();
        profile_follower_.resetSetpoint();
    }

    public Optional<Double> getYawGoalRadians() {
        return yaw_goal_.map(MotionProfileGoal::pos);
    }

    public void setYawGoal(Rotation2d goal) {
        if (yaw_goal_.isPresent() && !PoofsUtil.epsilonEquals(goal.getRadians(), yaw_goal_.get().pos())) {
            // If we get a changed yaw goal, reset the profile
            reset();
        }
        yaw_goal_ = Optional.of(new MotionProfileGoal(goal.getRadians()));
    }

}
