package frc.robot.subsystems.controllerFeedback;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.lib.dashboard.DashboardToggleSwitch;
import frc.robot.lib.feedback.PS5ControllerFeedbackHelper;
import frc.robot.lib.feedback.PS5ControllerFeedbackHelper.ConstantFeedback;
import frc.robot.lib.feedback.PS5ControllerFeedbackHelper.FeedbackPattern;
import frc.robot.lib.feedback.PS5ControllerFeedbackHelper.SinglePulseFeedback;
import frc.robot.lib.feedback.PS5ControllerFeedbackHelper.TimedFeedback;
import frc.robot.util.poofsUtils.TimeDelayedBoolean;
import frc.robot.util.poofsUtils.VirtualSubsystem;

public class ControllerFeedback extends VirtualSubsystem {
    private final PS5ControllerFeedbackHelper mDriver;
    private final PS5ControllerFeedbackHelper mOperator;

    public final DashboardToggleSwitch mEnableVibrationOffFMS;

    private final TimeDelayedBoolean mEndgame1Timer;
    private final TimeDelayedBoolean mEndgame2Timer;

    public ControllerFeedback(PS5Controller driver, PS5Controller operator) {
        mDriver = new PS5ControllerFeedbackHelper(driver);
        mOperator = new PS5ControllerFeedbackHelper(operator);

        mEnableVibrationOffFMS = new DashboardToggleSwitch("enableVibrationOffFMS", false, "No Vibration", "Use Vibration");

        mEndgame1Timer = new TimeDelayedBoolean();
        mEndgame2Timer = new TimeDelayedBoolean();
    }

    @Override
    public void periodic() {
        boolean endgame1 = false;
        boolean endgame2 = false;

        if (DriverStation.isTeleopEnabled() && (DriverStation.isFMSAttached() || mEnableVibrationOffFMS.getAsBoolean())) {
            if (endgame2 && !mEndgame1Timer.update(endgame2, 1)) {
                mDriver.setPattern(TimedFeedback.kEndgameWarning2);
                mOperator.setPattern(TimedFeedback.kEndgameWarning2);
            } else {
                mDriver.setPattern(ConstantFeedback.kNone);
                mOperator.setPattern(ConstantFeedback.kNone);
            }
        } else {
            setBothFeedback(ConstantFeedback.kNone);
        }

        mDriver.update();
        mOperator.update();
    }

    public void setDriverFeedback(FeedbackPattern pattern) {
        mDriver.setPattern(pattern);
    }

    public void setOperatorFeedback(FeedbackPattern pattern) {
        mOperator.setPattern(pattern);
    }

    public void setBothFeedback(FeedbackPattern pattern) {
        setDriverFeedback(pattern);
        setOperatorFeedback(pattern);
    }
}
