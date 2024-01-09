// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.dashboard.DashboardToggleSwitch;

/** Add your docs here. */
public class GripperIOSim implements GripperIO {
    private boolean mIntakeIsOccupied = false;

    private Timer mIntakeTimer;
    private final double kFakeIntakeDelay = 0.5;

    private double mCachedMotorSpeed = 0.0;

    public static final DashboardToggleSwitch mPreloadGripperInAuton = new DashboardToggleSwitch("preloadGripperSim", false, "No Preload", "Preload Gamepiece");

    private boolean mGripperAlreadyPreloaded = false;
    private boolean mWasAutonLastCycle = false;

    public GripperIOSim() {
        mIntakeTimer = new Timer();
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        if (DriverStation.isAutonomousEnabled()) {
            if (!mWasAutonLastCycle) {
                mGripperAlreadyPreloaded = false;
            }
            if (mPreloadGripperInAuton.getAsBoolean() && !mGripperAlreadyPreloaded) {
                mIntakeIsOccupied = true;
                mGripperAlreadyPreloaded = true;
            }
            mWasAutonLastCycle = true;
        } else {
            mWasAutonLastCycle = false;
        }

        if (mCachedMotorSpeed < 0) {
            if (!mIntakeIsOccupied) {
                mIntakeTimer.start();
                if (mIntakeTimer.get() >= kFakeIntakeDelay) {
                    mIntakeIsOccupied = true;
                }
            }
        } else if (mCachedMotorSpeed > 0) {
            if (mIntakeIsOccupied) {
                mIntakeTimer.start();
                if (mIntakeTimer.get() >= kFakeIntakeDelay) {
                    mIntakeIsOccupied = false;
                }
            }
        } else {
            mIntakeTimer.stop();
            mIntakeTimer.reset();
        }

        inputs.coneInIntake = mIntakeIsOccupied;
        inputs.cubeInIntake = mIntakeIsOccupied;
    }

    @Override
    public void setMotor(double throttle) {
        mCachedMotorSpeed = throttle;
    }
}
