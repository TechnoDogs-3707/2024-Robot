// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class LEDIOSim implements LEDIO {
    private final AddressableLED fakeLeds;
    private final AddressableLEDBuffer fakeBuffer;
    private final int numLEDs;

    public LEDIOSim(int length) {
        numLEDs = length + 8;
        fakeLeds = new AddressableLED(0);
        fakeLeds.setLength(numLEDs);
        fakeBuffer = new AddressableLEDBuffer(numLEDs);
        fakeLeds.setData(fakeBuffer);
        fakeLeds.start();
    }

    @Override
    public void setLEDs(int r, int g, int b, int w, int startID, int count) {
        // System.out.println("startid " + startID + " count " + count);
        for (int i = 0; i < count; i++) {
            fakeBuffer.setRGB(startID + i, r, g, b);
        }
        fakeLeds.setData(fakeBuffer);
    }

    @Override
    public void setLEDs(int r, int g, int b) {
        for (int i = 0; i < numLEDs; i++) {
            fakeBuffer.setRGB(i, r, g, b);
        }
        fakeLeds.setData(fakeBuffer);
    }
}
