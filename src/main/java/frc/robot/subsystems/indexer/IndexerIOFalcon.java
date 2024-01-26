// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import static frc.robot.Constants.Indexer.*;

/** Add your docs here. */
public class IndexerIOFalcon implements IndexerIO {
    private final TalonFX mMotor;
    private final TimeOfFlight mFirstSensor;
    private final TimeOfFlight mSecondSensor;

    public IndexerIOFalcon() {
        mMotor = new TalonFX(kMotorID, kMotorBus);
        mFirstSensor = new TimeOfFlight(40);
        mSecondSensor = new TimeOfFlight(41);


    }

    private void configSensors() {
        // mFirstSensor.setRangeOfInterest(0, 0, 0, 0);
        mFirstSensor.setRangingMode(RangingMode.Short, 24);
        
        // mSecondSensor.setRangeOfInterest(0, 0, 0, 0);
        mSecondSensor.setRangingMode(RangingMode.Short, 24);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        
    }

    @Override
    public void updateOutputs() {
        
    }

    @Override
    public void setMotorThrottle(double throttle) {
        
    }
}
