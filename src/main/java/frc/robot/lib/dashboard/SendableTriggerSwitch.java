// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;

/** Add your docs here. */
public class SendableTriggerSwitch implements Sendable, BooleanSupplier {
    private boolean mValue;
    private boolean mOldValue;
    public final EventLoop mAnyEdge;
    public final EventLoop mRisingEdge;
    public final EventLoop mFallingEdge;

    private final ArrayList<BooleanConsumer> mConsumers;

    public SendableTriggerSwitch() {
        this.mValue = false;
        this.mAnyEdge = new EventLoop();
        this.mRisingEdge = new EventLoop();
        this.mFallingEdge = new EventLoop();

        mConsumers = new ArrayList<>();
        mAnyEdge.bind(() -> mConsumers.forEach((c) -> c.accept(getAsBoolean())));
    }

    public void bind(BooleanConsumer... consumers) {
        for (BooleanConsumer consumer : consumers) {
            mConsumers.add(consumer);
        }
    }

    public boolean getmValue() {
        return mValue;
    }

    @Override
    public boolean getAsBoolean() {
        return getmValue();
    }

    public void setmValue(boolean value) {
        this.mOldValue = this.mValue;
        this.mValue = value;

        if (this.mValue && !this.mOldValue) {
            // rising edge
            mAnyEdge.poll();
            mRisingEdge.poll();
        }
        if (!this.mValue && this.mOldValue) {
            // falling edge
            mAnyEdge.poll();
            mFallingEdge.poll();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Value", this::getmValue, this::setmValue);        
    }
}
