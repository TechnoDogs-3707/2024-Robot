// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.ArrayList;
import java.util.Collection;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** A concretely typed wrapper class for dasboard widgets represented by a sendable object. */
public class SendableWidget<T extends Sendable> {

    private static final Collection<SendableWidget<?>> widgetsList = new ArrayList<>();

    /**
     * Create a new widget using the supplied sendable object.
     * @param tab The name of the shuffleboard tab to display this widget.
     * @param title The title of this widget.
     * @param sendable The sendable object to display.
     * @param config The widget configuration.
     */
    public SendableWidget(String tab, String title, T sendable, WidgetConfig config) {
        if (config.getUseBuiltInWidgets()) {
            Shuffleboard.getTab(tab).add(title, sendable)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getBuiltInWidget())
                .withProperties(config.getProperties());
        } else {
            Shuffleboard.getTab(tab).add(title, sendable)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getExternalWidget())
                .withProperties(config.getProperties());
        }

        widgetsList.add(this);
    }

    public static Collection<SendableWidget<?>> getWidgetslist() {
        return widgetsList;
    }

}
