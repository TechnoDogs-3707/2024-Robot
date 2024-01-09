package frc.robot.lib.leds;

import frc.robot.subsystems.leds.LEDIO;

public class LEDStateContainer implements ILEDDisplayable {
    LEDState staticState;
    AddressableLEDState addressableState;
    boolean inAddressableMode;

    public LEDStateContainer() {
        staticState = new LEDState();
        addressableState = new AddressableLEDState();
        inAddressableMode = false;
    }

    public void copyFrom(ILEDDisplayable other) {
        if (other instanceof LEDState) {
            staticState.copyFrom((LEDState)other);
            inAddressableMode = false;
        } else if (other instanceof AddressableLEDState) {
            addressableState.copyFrom((AddressableLEDState) other);
            inAddressableMode = true;
        } else if (other instanceof LEDStateContainer) {
            inAddressableMode = ((LEDStateContainer)other).inAddressableMode;
            staticState.copyFrom(((LEDStateContainer)other).staticState);
            addressableState.copyFrom(((LEDStateContainer)other).addressableState);
        }
    }

    @Override
    public void writePixels(LEDIO ledIO) {
        if (inAddressableMode) {
            this.addressableState.writePixels(ledIO);
        } else {
            this.staticState.writePixels(ledIO);
        }
    }
}
