// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.ArrayList;
import java.util.Collection;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** A concretely typed wrapper class for video stream dashboard widgets. */
public class WidgetVideoStream<T extends VideoSource> {

    private static final Collection<WidgetVideoStream<?>> widgetsList = new ArrayList<>();

    public WidgetVideoStream(String tab, String title, T stream, WidgetConfig config) {
        if (config.getUseBuiltInWidgets()) {
            Shuffleboard.getTab(tab).add(title, stream)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getBuiltInWidget())
                .withProperties(config.getProperties());
        } else {
            Shuffleboard.getTab(tab).add(title, stream)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getExternalWidget())
                .withProperties(config.getProperties());
        }
        widgetsList.add(this);
    }

    public static Collection<WidgetVideoStream<?>> getWidgetslist() {
        return widgetsList;
    }

}
