package frc.robot.subsystems.flywheels;

public class FlywheelsState {
    private boolean mFlywheelEnabled;
    private boolean mSlowIdle;
    private boolean mBrakeModeEnabled;

    public FlywheelsState(boolean flywheelEnabled, boolean slowIdle, boolean brakeModeEnabled) {
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
