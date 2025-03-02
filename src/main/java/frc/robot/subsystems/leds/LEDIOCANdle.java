// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import frc.robot.lib.phoenixpro.PhoenixErrorChecker;

/** Add your docs here. */
public class LEDIOCANdle implements LEDIO {
    private final CANdle candle;

    public LEDIOCANdle(int id, String bus) {
        candle = new CANdle(id, bus);
        CANdleConfiguration config = new CANdleConfiguration();
        config.disableWhenLOS = true;
        config.statusLedOffWhenActive = true;
        config.vBatOutputMode = VBatOutputMode.Off;

        PhoenixErrorChecker.checkErrorAndRetryV5(() -> candle.configAllSettings(config, 100));

        // PhoenixErrorChecker.checkErrorAndRetryV5(() -> candle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 250));
        // PhoenixErrorChecker.checkErrorAndRetryV5(() -> candle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10));
        // PhoenixErrorChecker.checkErrorAndRetryV5(() -> candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 250));
    }

    @Override
    public void updateInputs(LEDIOInputsAutoLogged inputs) {
        // inputs.inputVoltage = candle.getBusVoltage();
    }

    @Override
    public void setLEDs(int r, int g, int b, int w, int startID, int count) {
        candle.setLEDs(r, g, b, w, startID, count);
    }

    @Override
    public void setLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void setMasterBrightness(double brightness) {
        candle.configBrightnessScalar(brightness);
    }
}
