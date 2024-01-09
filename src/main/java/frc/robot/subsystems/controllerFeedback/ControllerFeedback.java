package frc.robot.subsystems.controllerFeedback;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotStateTracker;
import frc.robot.lib.dashboard.DashboardToggleSwitch;
import frc.robot.lib.feedback.XboxControllerFeedbackHelper;
import frc.robot.lib.feedback.XboxControllerFeedbackHelper.ConstantFeedback;
import frc.robot.lib.feedback.XboxControllerFeedbackHelper.FeedbackPattern;
import frc.robot.lib.feedback.XboxControllerFeedbackHelper.SinglePulseFeedback;
import frc.robot.lib.feedback.XboxControllerFeedbackHelper.TimedFeedback;
import frc.robot.lib.util.TimeDelayedBoolean;
import frc.robot.lib.util.VirtualSubsystem;

public class ControllerFeedback extends VirtualSubsystem {
    private final XboxControllerFeedbackHelper mDriver;
    private final XboxControllerFeedbackHelper mOperator;

    public final DashboardToggleSwitch mEnableVibrationOffFMS;

    private final TimeDelayedBoolean mEndgame1Timer;
    private final TimeDelayedBoolean mEndgame2Timer;

    public ControllerFeedback(XboxController driver, XboxController operator) {
        mDriver = new XboxControllerFeedbackHelper(driver);
        mOperator = new XboxControllerFeedbackHelper(operator);

        mEnableVibrationOffFMS = new DashboardToggleSwitch("enableVibrationOffFMS", false, "No Vibration", "Use Vibration");

        mEndgame1Timer = new TimeDelayedBoolean();
        mEndgame2Timer = new TimeDelayedBoolean();
    }

    @Override
    public void periodic() {
        boolean endgame1 = false;
        boolean endgame2 = false;
        boolean autoAlignOnTarget = RobotStateTracker.getInstance().getAutoAlignComplete();
        boolean autoAlignRunning = RobotStateTracker.getInstance().getAutoAlignActive();
        boolean autoAlignAvailable = RobotStateTracker.getInstance().getAutoAlignReady();

        if (DriverStation.isTeleopEnabled() && (DriverStation.isFMSAttached() || mEnableVibrationOffFMS.getAsBoolean())) {
            if (endgame2 && !mEndgame1Timer.update(endgame2, 1)) {
                mDriver.setPattern(TimedFeedback.kEndgameWarning2);
                mOperator.setPattern(TimedFeedback.kEndgameWarning2);
            }
            else if (endgame1 && !mEndgame2Timer.update(endgame1, 1)) {
                mDriver.setPattern(TimedFeedback.kEndgameWarning1);
                mOperator.setPattern(TimedFeedback.kEndgameWarning2);
            }
            else if (autoAlignOnTarget) {
                mDriver.setPattern(SinglePulseFeedback.kAlignmentOnTarget);
                mOperator.setPattern(ConstantFeedback.kNone);
            }
            else if (autoAlignRunning) {
                mDriver.setPattern(ConstantFeedback.kAlignmentRunning);
                mOperator.setPattern(ConstantFeedback.kNone);
            }
            else if (autoAlignAvailable) {
                mDriver.setPattern(TimedFeedback.kAlignmentReady);
                mOperator.setPattern(ConstantFeedback.kNone);
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
