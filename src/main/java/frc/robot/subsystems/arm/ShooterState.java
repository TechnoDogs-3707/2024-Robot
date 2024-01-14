package frc.robot.subsystems.arm;

public class ShooterState {
    private boolean mFlywheelEnabled;
    private boolean mSlowIdle;
    private boolean mBrakeModeEnabled;

    public ShooterState(boolean flywheelEnabled, boolean slowIdle, boolean brakeModeEnabled) {
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
