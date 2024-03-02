package frc.robot.lib.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class NoteAlignMotionPlanner {

    // private ProfileFollower mXController = new ProfileFollower(2.5, 0.0, 0.0, 1.0, 0.0, 0.0);
    // private ProfileFollower mYController = new ProfileFollower(2.5, 0.0, 0.0, 1.0, 0.0, 0.0);
    // private ProfileFollower mThetaController = new ProfileFollower(1.5, 0.0, 0.0, 1.0, 0.0, 0.0);

    private PIDController turnController = new PIDController(3, 0.01, 0.05);
    private PIDController forwardBackController = new PIDController(3, 0.01, 0.05); // NO IDEA WHAT THESE SHOULD BE
    boolean mAlignComplete = false;
    private double mXOffset = 0;
    private double mYOffset = 0;

    private OptionalDouble mStartTime;


    public void reset() {
        mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
        turnController.reset();
        turnController.setTolerance(mXOffset);
        turnController.setSetpoint(0); // change this if the camera is offset 
        forwardBackController.reset();
        forwardBackController.setTolerance(mYOffset);
        forwardBackController.setSetpoint(0); // change this if the camera is offset 
        mAlignComplete = false;
    }

    public synchronized void setCurrentXOffset(double offset) {
        mXOffset = offset;
    }

    public synchronized void setXTolerance(double tolerance) {
        turnController.setTolerance(tolerance);
    }

    public synchronized void setCurrentYOffset(double offset) {
        mXOffset = offset;
    }

    public synchronized void setYTolerance(double tolerance) {
        forwardBackController.setTolerance(tolerance);
    }

    public synchronized ChassisSpeeds updateAlignNote(double timestamp, ChassisSpeeds currentSpeeds, double currentXOffset, double currentYOffset) {

        double turnOutput = turnController.calculate(Rotation2d.fromDegrees(currentXOffset).getRadians());

        mAlignComplete = turnController.atSetpoint();

        return currentSpeeds.plus(new ChassisSpeeds(0,0, turnOutput));

    }

    public synchronized boolean getAlignComplete() {
        return mAlignComplete;
    }
}
