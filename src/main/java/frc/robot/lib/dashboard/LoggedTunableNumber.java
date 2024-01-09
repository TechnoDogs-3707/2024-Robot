// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.HashMap;
import java.util.Map;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
* Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
* value not in dashboard.
*/
public class LoggedTunableNumber {
    private static final String tableKey = "TunableNumbers";
    
    private final String mKey;
    private boolean mHasDefault = false;
    private double mDefaultValue;
    private LoggedDashboardNumber mDashboardNumber;
    private Map<Integer, Double> mLastHasChangedValues = new HashMap<>();
    
    /**
    * Create a new LoggedTunableNumber
    *
    * @param dashboardKey Key on dashboard
    */
    public LoggedTunableNumber(String dashboardKey) {
        mKey = tableKey + "/" + dashboardKey;
    }
    
    /**
    * Create a new LoggedTunableNumber with the default value
    *
    * @param dashboardKey Key on dashboard
    * @param defaultValue Default value
    */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }
    
    /**
    * Set the default value of the number. The default value can only be set once.
    *
    * @param defaultValue The default value
    */
    public void initDefault(double defaultValue) {
        if (!mHasDefault) {
            mHasDefault = true;
            mDefaultValue = defaultValue;
            if (Constants.tuningMode) {
                mDashboardNumber = new LoggedDashboardNumber(mKey, defaultValue);
            }
        }
    }
    
    /**
    * Get the current value, from dashboard if available and in tuning mode.
    *
    * @return The current value
    */
    public double get() {
        if (!mHasDefault) {
            return 0.0;
        } else {
            return Constants.tuningMode ? mDashboardNumber.get() : mDefaultValue;
        }
    }
    
    /**
    * Checks whether the number has changed since our last check
    *
    * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
    *     objects. Recommended approach is to pass the result of "hashCode()"
    * @return True if the number has changed since the last time this method was called, false
    *     otherwise.
    */
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = mLastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            mLastHasChangedValues.put(id, currentValue);
            return true;
        }
        
        return false;
    }
}