package frc.robot.lib.leds;

import frc.robot.subsystems.leds.LEDIO;

public interface ILEDDisplayable {
    void writePixels(LEDIO ledio);
}
