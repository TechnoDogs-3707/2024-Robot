// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.feedback;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/** Add your docs here. */
public class PS5ControllerFeedbackHelper {

    public enum FeedbackTypes {
        kCoarse,
        kFine,
        kBoth
    }

    public static class FeedbackOutput {
        public static final FeedbackOutput kNoOutput = new FeedbackOutput(FeedbackTypes.kBoth, 0);
        public static final FeedbackOutput kAlignmentReady = new FeedbackOutput(FeedbackTypes.kFine, 0.125);
        public static final FeedbackOutput kAlignmentRunning = new FeedbackOutput(FeedbackTypes.kFine, 0.0625);
        public static final FeedbackOutput kAlignmentOnTarget = new FeedbackOutput(FeedbackTypes.kCoarse, 1);

        public static final FeedbackOutput kEndgameWarning1 = new FeedbackOutput(FeedbackTypes.kCoarse, 0.5);
        public static final FeedbackOutput kEndgameWarning2 = new FeedbackOutput(FeedbackTypes.kCoarse, 1);

        public final FeedbackTypes mFeedbackType;
        public final double mFeedbackStrength;

        public FeedbackOutput(FeedbackTypes type, double strength) {
            mFeedbackType = type;
            mFeedbackStrength = strength;
        }
    }

    public interface FeedbackPattern {
        public FeedbackOutput getOutput(double timeSinceStart);
    }

    public static class ConstantFeedback implements FeedbackPattern {
        public static final ConstantFeedback kNone = new ConstantFeedback(FeedbackOutput.kNoOutput);
        public static final ConstantFeedback kAlignmentRunning = new ConstantFeedback(FeedbackOutput.kAlignmentRunning);

        private final FeedbackOutput mOutput;

        public ConstantFeedback(FeedbackOutput output) {
            mOutput = output;
        }

        @Override
        public FeedbackOutput getOutput(double timeSinceStart) {
            return mOutput;
        }
    }

    public static class TimedFeedback implements FeedbackPattern {
        public static final TimedFeedback kAlignmentReady = new TimedFeedback(0.125, 0.25, FeedbackOutput.kAlignmentReady, FeedbackOutput.kNoOutput);
        public static final TimedFeedback kEndgameWarning1 = new TimedFeedback(0.5, 0.5, FeedbackOutput.kEndgameWarning1, FeedbackOutput.kNoOutput);
        public static final TimedFeedback kEndgameWarning2 = new TimedFeedback(0.75, 0.25, FeedbackOutput.kEndgameWarning2, FeedbackOutput.kNoOutput);

        private final double mTimeOn;
        private final double mTimeOff;
        private final FeedbackOutput mOutputWhenOn;
        private final FeedbackOutput mOutputWhenOff;

        public TimedFeedback(double timeOn, double timeOff, FeedbackOutput outputWhenOn, FeedbackOutput outputWhenOff) {
            mTimeOn = timeOn;
            mTimeOff = timeOff;
            mOutputWhenOn = outputWhenOn;
            mOutputWhenOff = outputWhenOff;
        }

        public final FeedbackOutput getOutput(double timeSincePatternStart) {
            if(mTimeOn != mTimeOff) {
                if (timeSincePatternStart % (mTimeOn + mTimeOff) > mTimeOn) {
                    return mOutputWhenOn;
                } else {
                    return mOutputWhenOff;
                }
            } else {
                if ((int) (timeSincePatternStart / mTimeOff) % 2 == 0) {
                    return mOutputWhenOn;
                } else {
                    return mOutputWhenOff;
                }
            }
        }
    }

    public static class SinglePulseFeedback implements FeedbackPattern {
        public static final SinglePulseFeedback kAlignmentOnTarget = new SinglePulseFeedback(0.125, FeedbackOutput.kAlignmentOnTarget, FeedbackOutput.kNoOutput);

        private final double mPulseTime;
        private final FeedbackOutput mOutputDuringPulse;
        private final FeedbackOutput mOutputAfterPulse;

        public SinglePulseFeedback(double pulseTime, FeedbackOutput outputDuringPulse, FeedbackOutput outputAfterPulse) {
            mPulseTime = pulseTime;
            mOutputDuringPulse = outputDuringPulse;
            mOutputAfterPulse = outputAfterPulse;
        }

        @Override
        public FeedbackOutput getOutput(double timeSinceStart) {
            if (timeSinceStart < mPulseTime) {
                return mOutputDuringPulse;
            }
            return mOutputAfterPulse;
        }
    }

    private final PS5Controller mController;
    private double mPatternStarted;
    private FeedbackPattern mCurrentPattern;

    public PS5ControllerFeedbackHelper(PS5Controller controller) {
        mController = controller;
        mPatternStarted = 0.0;
        mCurrentPattern = ConstantFeedback.kNone;
    }

    public void setPattern(FeedbackPattern pattern) {
        if (pattern != mCurrentPattern) {
            mCurrentPattern = pattern;
            mPatternStarted = Timer.getFPGATimestamp();
        }
    }

    public void update() {
        double now = Timer.getFPGATimestamp();
        double timeSinceStart = now - mPatternStarted;
        
        setOutput(mCurrentPattern.getOutput(timeSinceStart));
    }

    private void setOutput(FeedbackOutput output) {
        RumbleType rumbleType = RumbleType.kBothRumble;

        switch (output.mFeedbackType) {
            case kBoth:
                rumbleType = RumbleType.kBothRumble;
                break;
            case kCoarse:
                rumbleType = RumbleType.kLeftRumble;
                break;
            case kFine:
                rumbleType = RumbleType.kRightRumble;
                break;
            default:
                DriverStation.reportWarning("GenericHID Rumble Type conversion fell through.", false);
                break;
        }

        mController.setRumble(rumbleType, output.mFeedbackStrength);
    }

    public void stopOutput() {
        setPattern(ConstantFeedback.kNone);
        update();
    }
}
