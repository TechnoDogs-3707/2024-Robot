// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** 
 * A singleton class for sharing robot state information between subsystems.
 */
public class RobotStateTracker {
    private Translation2d mCurrentRobotPosition = new Translation2d();
    private ChassisSpeeds mCurrentRobotSpeed = new ChassisSpeeds();
    private Pose2d mCurrentRobotPose = new Pose2d();

    private boolean mAutoAlignReady = false;
    private boolean mAutoAlignActive = false;
    private boolean mAutoAlignComplete = false;

    public static final RobotStateTracker instance = new RobotStateTracker();

    public static RobotStateTracker getInstance() {
        return instance;
    }

    private RobotStateTracker() {

    }

    public Translation2d getCurrentRobotPosition() {
        return mCurrentRobotPosition;
    }

    public ChassisSpeeds getCurrentRobotSpeeds() {
        return mCurrentRobotSpeed;
    }

    public Pose2d getCurrentRobotPose() {
        return mCurrentRobotPose;
    }

    public boolean getAutoAlignReady() {
        return mAutoAlignReady;
    }

    public boolean getAutoAlignActive() {
        return mAutoAlignActive;
    }

    public boolean getAutoAlignComplete() {
        return mAutoAlignComplete;
    }

    public void setCurrentRobotPosition(Translation2d position) {
        mCurrentRobotPosition = position;
    }

    public void setCurrentRobotVelocity(ChassisSpeeds speeds) {
        mCurrentRobotSpeed = speeds;
    }

    public void setCurrentRobotPose(Pose2d mCurrentRobotPose) {
        this.mCurrentRobotPose = mCurrentRobotPose;
    }

    public void setAutoAlignReady(boolean autoAlignReady) {
        mAutoAlignReady = autoAlignReady;
    }

    public void setAutoAlignActive(boolean mAutoAlignActive) {
        this.mAutoAlignActive = mAutoAlignActive;
    }

    public void setAutoAlignComplete(boolean mAutoAlignComplete) {
        this.mAutoAlignComplete = mAutoAlignComplete;
    }
}
