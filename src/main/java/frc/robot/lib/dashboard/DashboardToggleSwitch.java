package frc.robot.lib.dashboard;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DashboardToggleSwitch implements BooleanSupplier {
    private final LoggedDashboardChooser<Boolean> mChooser;
    private Trigger mTrigger;
    private boolean mDefaultValue;

    public DashboardToggleSwitch(String key, boolean defaultValue) {
        this(key, defaultValue, "False", "True");
    }

    public DashboardToggleSwitch(String key, boolean defaultValue, String titleWhenOff, String titleWhenOn) {
        mChooser = new LoggedDashboardChooser<>(key);
        if (defaultValue) {
            mChooser.addOption(titleWhenOff, false);
            mChooser.addDefaultOption(titleWhenOn, true);
        } else {
            mChooser.addDefaultOption(titleWhenOff, false);
            mChooser.addOption(titleWhenOn, true);
        }
        mDefaultValue = defaultValue;
    }

    @Override
    public boolean getAsBoolean() {
        Boolean val = mChooser.get();
        if (val == null) {
            return mDefaultValue; 
        } else {
            return val;
        }
    }

    public Trigger asTrigger() {
        if (mTrigger == null) {
            mTrigger = new Trigger(this);
        }
        return mTrigger;
    }

    public SendableChooser<String> getChooser() {
        return mChooser.getSendableChooser();
    }
}
