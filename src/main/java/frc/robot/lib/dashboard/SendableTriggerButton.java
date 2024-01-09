// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** A sendable that can be put on a dashboard to run a function when a button is pressed.
 * Uses the same widget as a WPILib Command.
*/
public class SendableTriggerButton implements Sendable {
    private Runnable mCallback;
    private boolean mTrigged;
    private String mButtonText;


    /**
     * Creates a new instance of a Sendable Button.
     * @param buttonText The text to display on the button.
     * @param callback The runnable to execute when the button is pressed.
     */
    public SendableTriggerButton(String buttonText, Runnable callback) {
        mCallback = callback;
        mButtonText = buttonText;
    }

    /**
     * Get the current button text.
     * @return The text currently displayed in the button.
     */
    public String getmButtonText() {
        return mButtonText;
    }

    /**
     * Check if the button is currently pressed.
     * @return True if the button is pressed.
     */
    public boolean ismTrigged() {
        return mTrigged;
    }

    public void setmTrigged(boolean triggered) {
        if (mTrigged && triggered) {
            mTrigged = true;
        } else if (mTrigged && !triggered) {
            mTrigged = false;
        } else if (!mTrigged && triggered) {
            mTrigged = true;
            mCallback.run();
            mTrigged = false;
        } else if (!mTrigged && !triggered) {
            mTrigged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Command");
        builder.addStringProperty(".name", this::getmButtonText, null);
        builder.addBooleanProperty("running", this::ismTrigged, this::setmTrigged);
    }}
