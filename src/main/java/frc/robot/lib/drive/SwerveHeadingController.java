package frc.robot.lib.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.Utility;

/**
 * Controls overall swerve heading of the robot through motion profile.
 * <p>
 * All units are in degrees (for this class only) for easy integration with DPad
 */
public class SwerveHeadingController {
    private static SwerveHeadingController mInstance;
    // private RobotState mRobotState = RobotState.getInstance();

    public static SwerveHeadingController getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveHeadingController();
        }

        return mInstance;
    }

    public Pose2d centerOfGoal;

    private final Pose2d kSpeakerLocationRed = new Pose2d(16.75, 5.56, Rotation2d.fromDegrees(180));
    private final Pose2d kSpeakerLocationBlue = new Pose2d(0, 5.56, Utility.R2D_IDENTITY);

    public enum HeadingControllerState {
        OFF, SNAP, // for snapping to specific headings
        MAINTAIN, // maintaining current heading while driving
        POLAR_MAINTAIN, // for maintaining heading toward origin
        POLAR_SNAP, // for snapping heading toward origin
    }

    private final PIDController mPIDFController;
    private double mSetpoint = 0.0;

    private HeadingControllerState mHeadingControllerState = HeadingControllerState.OFF;

    private SwerveHeadingController() {
        mPIDFController = new PIDController(0, 0, 0);
    }

    public HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    /**
     * @param goal_pos pos in degrees
     */
    public void setGoal(Double goal_pos) {
        mSetpoint = goal_pos;
    }

    public double getGoal() {
        return mSetpoint;
    }

    public boolean isAtGoal() {
        // return mPIDFController.onTarget(Constants.kSwerveHeadingControllerErrorTolerance);
        return Math.abs(mPIDFController.getPositionError()) <= Constants.kSwerveHeadingControllerErrorTolerance;
    }

    public double getAbsError() {
        return Math.abs(mPIDFController.getPositionError());
    }

    public double calculateAngleToOrigin(Pose2d current_pose) {
        // centerOfGoal = mRobotState.getFieldToGoal();
        centerOfGoal = new Pose2d();
        double r = current_pose.getTranslation().getDistance(Utility.TR2D_IDENTITY);
        double theta = Math.atan2(current_pose.getTranslation().getY(), current_pose.getTranslation().getX());
        double r_central = centerOfGoal.getTranslation().getDistance(Utility.TR2D_IDENTITY);
        double theta_central = Math.atan2(centerOfGoal.getTranslation().getY(), centerOfGoal.getTranslation().getX());

        double angle = Math.toDegrees(Math.PI + Math.atan2(r * Math.sin(theta) - r_central * Math.sin(theta_central),
                r * Math.cos(theta) - r_central * Math.cos(theta_central)));
        if(angle < 0) {
            return -angle;
        }
        return angle;
    }

    public double calculateAngleToSpeaker(Pose2d current_pose) {
        // Pose2d speakerLocation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? kSpeakerLocationRed : kSpeakerLocationBlue;
        // return current_pose.relativeTo(speakerLocation).getRotation().unaryMinus().getDegrees();

        Pose2d speakerLocation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? kSpeakerLocationRed : kSpeakerLocationBlue;

        return current_pose.relativeTo(speakerLocation).getTranslation().getAngle().getDegrees();
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update(double current_angle) {
        mPIDFController.setSetpoint(mSetpoint);
        double current_error = mSetpoint - current_angle;

        if (current_error > 180) {
            current_angle += 360;
        } else if (current_error < -180) {
            current_angle -= 360;
        }

        // var current_translational_velocity = RobotState.getInstance().getMeasuredVelocity().norm();
        var current_translational_velocity = RobotState.getInstance().linearVelocity().getNorm();
        final double kMinTranslationalVelocity = 0.2;
        if (current_translational_velocity < kMinTranslationalVelocity) {
            current_translational_velocity = kMinTranslationalVelocity;
        }
        final double kMaxTranlationalVelocity = 2.5;
        if (current_translational_velocity > kMaxTranlationalVelocity) {
            current_translational_velocity = kMaxTranlationalVelocity;
        }
        // double interp = (current_translational_velocity - kMinTranslationalVelocity) / kMaxTranlationalVelocity;

        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDFController.setPID(Constants.kSnapSwerveHeadingKp, Constants.kSnapSwerveHeadingKi, Constants.kSnapSwerveHeadingKd);
                break;
            case MAINTAIN:
//                var kp = interp * (Constants.kMaintainSwerveHeadingKpHighVelocity - Constants.kMaintainSwerveHeadingKpLowVelocity) + Constants.kMaintainSwerveHeadingKpLowVelocity;
//                var ki = interp * (Constants.kMaintainSwerveHeadingKiHighVelocity - Constants.kMaintainSwerveHeadingKiLowVelocity) + Constants.kMaintainSwerveHeadingKiLowVelocity;
//                var kd = interp * (Constants.kMaintainSwerveHeadingKdHighVelocity - Constants.kMaintainSwerveHeadingKdLowVelocity) + Constants.kMaintainSwerveHeadingKdLowVelocity;
                mPIDFController.setPID(Constants.kMaintainSwerveHeadingKpHighVelocity, 0, Constants.kMaintainSwerveHeadingKdHighVelocity);
                // mPIDFController.setOutputRange(-1.0, 1.0);
                break;
            case POLAR_MAINTAIN:
                // mPIDFController.setPID(Constants.kMaintainSwerveHeadingKp, Constants.kMaintainSwerveHeadingKi, Constants.kMaintainSwerveHeadingKd);
                break;
            case POLAR_SNAP:
                mPIDFController.setPID(Constants.kSnapSwerveHeadingKp, Constants.kSnapSwerveHeadingKi, Constants.kSnapSwerveHeadingKd);
                break;
        }

        return mPIDFController.calculate(current_angle);
    }
}