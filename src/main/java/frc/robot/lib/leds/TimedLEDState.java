package frc.robot.lib.leds;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.poofsUtils.PoofsUtil;

public interface TimedLEDState {
    void getCurrentLEDState(LEDStateContainer desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kHandoffRunning = new BlinkingLEDState(LEDState.kPurple, LEDState.kOrange, 0.1);
        
        public static BlinkingLEDState kIntakeAutoAlignOnTarget = new BlinkingLEDState(LEDState.kOff, LEDState.kBlue, 0.1);
        public static BlinkingLEDState kAutoAimPreparing = new BlinkingLEDState(LEDState.kOff, LEDState.kYellow, 0.1);
        public static BlinkingLEDState kAutoAimScoring = new BlinkingLEDState(LEDState.kOff, LEDState.kWhite, 0.1);
        public static BlinkingLEDState kAutoAlignScoring = new BlinkingLEDState(LEDState.kOff, LEDState.kWhite, 0.1);

        public static BlinkingLEDState kIntakingToIndexer = new BlinkingLEDState(LEDState.kOff, LEDState.kOrange, 0.1);
        public static BlinkingLEDState kIntakingToArm = new BlinkingLEDState(LEDState.kOff, LEDState.kPurple, 0.1);

        public static BlinkingLEDState kClimbRunningDutyCycle = new BlinkingLEDState(LEDState.kOff, LEDState.kYellow, 0.1);

        public static BlinkingLEDState kVisionMissing = new BlinkingLEDState(LEDState.kOff, LEDState.kYellow, 0.1);

        LEDState mStateOne = new LEDState(0, 0, 0);
        LEDState mStateTwo = new LEDState(0, 0, 0);
        private boolean mAsymmetricDuration = false;
        private double mDuration;
        private double mDurationTwo;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double durationOne, double durationTwo) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = durationOne;
            mDurationTwo = durationTwo;
            mAsymmetricDuration = true;
        }

        @Override
        public void getCurrentLEDState(LEDStateContainer desiredState, double timestamp) {
            if(mAsymmetricDuration) {
                if (timestamp % (mDuration + mDurationTwo) > mDuration) {
                    desiredState.copyFrom(mStateTwo);
                } else {
                    desiredState.copyFrom(mStateOne);
                }
            } else {
                if ((int) (timestamp / mDuration) % 2 == 0) {
                    desiredState.copyFrom(mStateOne);
                } else {
                    desiredState.copyFrom(mStateTwo);
                }
            }
        }
    }

    class PercentFullLEDState implements TimedLEDState {
        AddressableLEDState mState;

        public PercentFullLEDState(double percentFull, LEDState fullColor) {
            LEDState[] pixels = new LEDState[Constants.kMaxLEDCount / 2];
            for (int i = 0; i < pixels.length; i++) {
                if (i < pixels.length * PoofsUtil.limit(percentFull, 0.0, 1.0)) {
                    pixels[i] = fullColor;
                }
            }
            mState = new AddressableLEDState(pixels).mirrored();
        }

        @Override
        public void getCurrentLEDState(LEDStateContainer desiredState, double timestamp) {
            desiredState.copyFrom(mState);
        }
    }

    class RSLBasedLEDState implements TimedLEDState {
        public static RSLBasedLEDState kVisionMissing = new RSLBasedLEDState(LEDState.kOff, LEDState.kYellow);
        public static RSLBasedLEDState kVisionPresent = new RSLBasedLEDState(LEDState.kOff, LEDState.kGreen);

        public static RSLBasedLEDState kIndexerEmpty = new RSLBasedLEDState(LEDState.kOff, LEDState.kOrange);

        public static RSLBasedLEDState kClimbRaising = new RSLBasedLEDState(LEDState.kOff, LEDState.kBlue);
        public static RSLBasedLEDState kClimbManual = new RSLBasedLEDState(LEDState.kRed, LEDState.kYellow);

        public static RSLBasedLEDState kAutoAimWaiting = new RSLBasedLEDState(LEDState.kOff, LEDState.kGreen);

        private final double mDurationIfSimulating = 0.2;
        LEDState mStateWhenOff = new LEDState(0, 0, 0);
        LEDState mStateWhenOn = new LEDState(0, 0, 0);

        public RSLBasedLEDState(LEDState stateWhenOff, LEDState stateWhenOn) {
            mStateWhenOff.copyFrom(stateWhenOff);
            mStateWhenOn.copyFrom(stateWhenOn);
        }

        @Override
        public void getCurrentLEDState(LEDStateContainer desiredState, double timestamp) {
            if (Robot.isSimulation()) {
                if ((int) (timestamp / mDurationIfSimulating) % 2 == 0) {
                    desiredState.copyFrom(mStateWhenOn);
                } else {
                    desiredState.copyFrom(mStateWhenOff);
                }
            } else {
                if (RobotController.getRSLState()) {
                    desiredState.copyFrom(mStateWhenOn);
                } else {
                    desiredState.copyFrom(mStateWhenOff);
                }
            }
        }
        
        
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kStaticRobotZeroedWithGoodBattery = new StaticLEDState(LEDState.kRobotZeroedWithGoodBattery);
        public static StaticLEDState kStaticBatteryLow = new StaticLEDState(LEDState.kBatteryLow);
        public static StaticLEDState kStaticNotHomed = new StaticLEDState(LEDState.kYellow);
        public static StaticLEDState kVisionDisabled = new StaticLEDState(LEDState.kRed);

        public static StaticLEDState kNoteInIndexer = new StaticLEDState(LEDState.kOrange);
        public static StaticLEDState kNoteInArm = new StaticLEDState(LEDState.kPurple);

        public static StaticLEDState kAutoAlignScoringComplete = new StaticLEDState(LEDState.kGreen);
        public static StaticLEDState kClimbHeightTargetReached = new StaticLEDState(LEDState.kGreen);

        LEDState mStaticState = new LEDState(0, 0, 0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDStateContainer desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }
}
