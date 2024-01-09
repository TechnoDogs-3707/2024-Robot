// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface LEDIO {
    @AutoLog
    public class LEDIOInputs {
        double inputVoltage = 0.0;
    }

    public default void updateInputs(LEDIOInputsAutoLogged inputs) {}

    public default void setLEDs(int r, int g, int b, int w, int startID, int count) {}

    public default void setLEDs(int r, int g, int b) {}

    public default void setMasterBrightness(double brightness) {}
}
