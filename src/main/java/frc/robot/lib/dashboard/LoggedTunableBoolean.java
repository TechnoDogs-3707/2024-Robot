// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import frc.robot.Constants;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

/**
* Class for a tunable boolean. Gets value from dashboard in tuning mode, returns default if not or
* value not in dashboard.
*/
public class LoggedTunableBoolean {
    private static final String tableKey = "TunableBooleans";
    
    private final String mKey;
    private boolean mHasDefault = false;
    private boolean mDefaultValue;
    private LoggedDashboardBoolean mDashboardBoolean;
    private Map<Integer, Boolean> mLastHasChangedValues = new HashMap<>();
    
    /**
    * Create a new LoggedTunableNumber
    *
    * @param dashboardKey Key on dashboard
    */
    public LoggedTunableBoolean(String dashboardKey) {
        mKey = tableKey + "/" + dashboardKey;
    }
    
    /**
    * Create a new LoggedTunableNumber with the default value
    *
    * @param dashboardKey Key on dashboard
    * @param defaultValue Default value
    */
    public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }
    
    /**
    * Set the default value of the number. The default value can only be set once.
    *
    * @param defaultValue The default value
    */
    public void initDefault(boolean defaultValue) {
        if (!mHasDefault) {
            mHasDefault = true;
            mDefaultValue = defaultValue;
            if (Constants.tuningMode) {
                mDashboardBoolean = new LoggedDashboardBoolean(mKey, defaultValue);
            }
        }
    }
    
    /**
    * Get the current value, from dashboard if available and in tuning mode.
    *
    * @return The current value
    */
    public boolean get() {
        if (!mHasDefault) {
            return false;
        } else {
            return Constants.tuningMode ? mDashboardBoolean.get() : mDefaultValue;
        }
    }
    
    /**
    * Checks whether the boolean has changed since our last check
    *
    * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
    *     objects. Recommended approach is to pass the result of "hashCode()"
    * @return True if the number has changed since the last time this method was called, false
    *     otherwise.
    */
    public boolean hasChanged(int id) {
        boolean currentValue = get();
        Boolean lastValue = mLastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            mLastHasChangedValues.put(id, currentValue);
            return true;
        }
        
        return false;
    }

    @SafeVarargs
    public final void ifChanged(int id, Consumer<Boolean>... consumer) {
        if (hasChanged(id)) {
            for (Consumer<Boolean> c : consumer) {
                c.accept(get());
            }
        }
    }
}