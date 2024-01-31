package frc.robot.subsystems.shooterFlywheels;

public class ShooterFlywheelsState {
    private boolean mFlywheelEnabled;
    private boolean mSlowIdle;
    private boolean mBrakeModeEnabled;

    public ShooterFlywheelsState(boolean flywheelEnabled, boolean slowIdle, boolean brakeModeEnabled) {
        mFlywheelEnabled = flywheelEnabled;
        mSlowIdle = slowIdle;
        mBrakeModeEnabled = brakeModeEnabled;
    }

    public boolean getFlywheelEnabled() {
        return mFlywheelEnabled;
    }

    public boolean getSlowIdle() {
        return mSlowIdle;
    }

    public boolean getBrakeModeEnabled() {
        return mBrakeModeEnabled;
    }
}
