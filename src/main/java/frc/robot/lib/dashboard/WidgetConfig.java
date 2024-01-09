// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/** Represents the configuration for a dashboard widget.
 * This class exists to make sure that all dashboard widgets have concrete position, size and properties.
 */
public class WidgetConfig {
    private int row, column, width, height;
    private Boolean useBuiltInWidgets;
    private BuiltInWidgets builtInWidget;
    private String externalWidget;
    private Map<String, Object> properties;

    private WidgetConfig(int right, int down, int width, int height, Boolean useBuiltInWidgets, BuiltInWidgets builtInWidget, String externalWidget, Map<String, Object> properties) {
        this.row = down;
        this.column = right;
        this.width = width;
        this.height = height;
        this.useBuiltInWidgets = useBuiltInWidgets;
        this.builtInWidget = builtInWidget;
        this.externalWidget = externalWidget;
        this.properties = properties;
    }

    /**
     * Create a new widget using a built in widget type and a specified configuration.
     * @param right The distance right to move the widget, starting at 0.
     * @param down The distance down to move the widget, starting at 0.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param widget The widget type to use.
     * @param properties A Map of the widget properties.
     */
    public WidgetConfig(int right, int down, int width, int height, BuiltInWidgets widget, Map<String, Object> properties) {
        this(right, down, width, height, true, widget, null, properties);
    }

    /**
     * Create a new widget using a built in widget type and no specified configuration.
     * @param right The distance right to move the widget, starting at 0.
     * @param down The distance down to move the widget, starting at 0.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param widget The widget type to use.
     */
    public WidgetConfig(int right, int down, int width, int height, BuiltInWidgets widget) {
        this(right, down, width, height, true, widget, null, null);
    }

    /**
     * Create a new widget using an external widget type with a specified configuration.
     * @param right The distance right to move the widget, starting at 0.
     * @param down The distance down to move the widget, starting at 0.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param externalWidget The name of the custom widget to use.
     * @param properties A Map of the widget properties.
     */
    public WidgetConfig(int right, int down, int width, int height, String externalWidget, Map<String, Object> properties) {
        this(right, down, width, height, false, null, externalWidget, properties);
    }

    /**
     * Create a new widget using an external widget type and no specified configuration.
     * @param right The distance right to move the widget, starting at 0.
     * @param down The distance down to move the widget, starting at 0.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param externalWidget The name of the custom widget to use.
     */
    public WidgetConfig(int right, int down, int width, int height, String externalWidget) {
        this(right, down, width, height, false, null, externalWidget, null);
    }

    /**
     * Get the current row of the widget's origin.
     * @return The widget's row
     */
    public int getRow() {
        return row;
    }

    /**
     * Get the current column of the widget's origin.
     * @return The widget's column.
     */
    public int getColumn() {
        return column;
    }

    /**
     * Get the current width of the widget.
     * @return The widget's width.
     */
    public int getWidth() {
        return width;
    }

    /**
     * Get the current height of the widget.
     * @return The widget's height.
     */
    public int getHeight() {
        return height;
    }

    public Boolean getUseBuiltInWidgets() {
        return useBuiltInWidgets;
    }

    public BuiltInWidgets getBuiltInWidget() {
        return builtInWidget;
    }
    
    public String getExternalWidget() {
        return externalWidget;
    }

    /**
     * Get the properties of a widget.
     * @return The widget's properties.
     */
    public Map<String, Object> getProperties() {
        return properties;
    }

}
