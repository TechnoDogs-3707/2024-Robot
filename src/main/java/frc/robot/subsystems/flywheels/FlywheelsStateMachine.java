package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.*;

// import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.Util;

public class FlywheelsStateMachine {
    public enum FlywheelsWantedAction {
        /**
         * Shooting motors are off, and in brake mode to prevent the indexer
         * from pushing a ring through it.
         */
        OFF,

        /**
         * Shooting motors are set to a constant slow speed setting to decrease spin-up
         * time. This should be set after the gamepiece is fully indexed.
         */
        IDLE,

        /**
         * Shooting motors are commanded to spin up to their target RPM.
         * Set this mode when within short distance of the scoring point to
         * waste less energy keeping the flywheel running.
         */
        SHOOT
    }

    public enum FlywheelsSystemState {
        /**
         * The shooter motors are disabled and in brake mode.
         */
        OFF,

        /**
         * The shooter motors are spinning slowly to help with spin-up time.
         */
        IDLE,

        /**
         * The shooter motors have been commanded to their target RPM,
         * but are not within tolerance.
         */
        SPINUP,

        /**
         * The shooter motors are within tolerance of their commanded RPM.
         */
        READY,

        /**
         * Shooter motors are disabled but still spinning.
         */
        SPINDOWN
    }

    private FlywheelsSystemState mSystemState = FlywheelsSystemState.OFF;
    private FlywheelsWantedAction mWantedAction = FlywheelsWantedAction.OFF;
    // private double mStateStartTime = Timer.getFPGATimestamp();

    public void setWantedAction(FlywheelsWantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
            // mStateStartTime = Timer.getFPGATimestamp();
        }
    }

    public FlywheelsSystemState getSystemState() {
        return mSystemState;
    }

    protected FlywheelsState update(double leftMotorRPS, double rightMotorRPS, double leftSetpointRPS, double rightSetpointRPS) {
        // double timeInState = Timer.getFPGATimestamp() - mStateStartTime;

        boolean shouldEnableFlywheel = false;
        boolean shouldSlowIdle = false;
        boolean shouldEnableBrakeMode = false;

        double fastestMotorRPS = Math.max(leftMotorRPS, rightMotorRPS);

        switch (mWantedAction) {
            case OFF:
                if (fastestMotorRPS > kMaxRPSForBrakeMode) {
                    mSystemState = FlywheelsSystemState.SPINDOWN;
                } else {
                    mSystemState = FlywheelsSystemState.OFF;
                }
                break;
            case IDLE:
                if (fastestMotorRPS > kMaxRPSForIdleControl) {
                    mSystemState = FlywheelsSystemState.SPINDOWN;
                } else {
                    mSystemState = FlywheelsSystemState.IDLE;
                }
                break;
            case SHOOT:
                if (Util.epsilonEquals(leftMotorRPS, leftSetpointRPS, kRPSTolerance) && Util.epsilonEquals(rightMotorRPS, rightSetpointRPS, kRPSTolerance)) {
                    mSystemState = FlywheelsSystemState.READY;
                } else {
                    mSystemState = FlywheelsSystemState.SPINUP;
                }
                break;
            default:
                mSystemState = FlywheelsSystemState.OFF;
                break;
        }

        switch (mSystemState) {
            case OFF:
                shouldEnableFlywheel = false;
                shouldSlowIdle = false;
                shouldEnableBrakeMode = true;
                break;
            case IDLE:
                shouldEnableFlywheel = true;
                shouldSlowIdle = true;
                shouldEnableBrakeMode = false;
                break;
            case SPINUP:
            case READY:
                shouldEnableFlywheel = true;
                shouldSlowIdle = false;
                shouldEnableBrakeMode = false;
                break;
            case SPINDOWN:
            default:
                shouldEnableFlywheel = false;
                shouldSlowIdle = false;
                shouldEnableBrakeMode = false;
                break;
        }

        return new FlywheelsState(shouldEnableFlywheel, shouldSlowIdle, shouldEnableBrakeMode);
    }
}
