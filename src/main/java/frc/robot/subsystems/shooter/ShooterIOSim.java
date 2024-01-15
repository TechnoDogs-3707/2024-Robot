// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim mLeftFlywheelSim;
    private final FlywheelSim mRightFlywheelSim;

    private final SimpleMotorFeedforward mShooterMotorFeedforward;

    private double mLeftSetpoint = 0.0;
    private double mRightSetpoint = 0.0;

    private boolean mSpinDownMode = false;

    public ShooterIOSim() {
        mLeftFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.0025);
        mRightFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.0025);

        mShooterMotorFeedforward = new SimpleMotorFeedforward(0, 0.11);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (mSpinDownMode) {
            mLeftFlywheelSim.setInputVoltage(0);
            mRightFlywheelSim.setInputVoltage(0);
        } else {
            mLeftFlywheelSim.setInputVoltage(mShooterMotorFeedforward.calculate(mLeftSetpoint));
            mRightFlywheelSim.setInputVoltage(mShooterMotorFeedforward.calculate(mRightSetpoint));
        }

        mLeftFlywheelSim.update(Constants.loopPeriodSecs);
        mRightFlywheelSim.update(Constants.loopPeriodSecs);

        inputs.leftMotorSpeedRPS = mLeftFlywheelSim.getAngularVelocityRPM() / 60.0;
        inputs.leftMotorSuppliedCurrentAmps = Math.abs(mLeftFlywheelSim.getCurrentDrawAmps());
        inputs.leftMotorTempCelsius = 0.0;

        inputs.rightMotorSpeedRPS = mRightFlywheelSim.getAngularVelocityRPM() / 60.0;
        inputs.rightMotorSuppliedCurrentAmps = Math.abs(mRightFlywheelSim.getCurrentDrawAmps());
        inputs.rightMotorTempCelsius = 0.0;
    }

    @Override
    public void setSpeedSetpointLeft(double setpointSpeedRPS) {
        mLeftSetpoint = setpointSpeedRPS;
    }

    @Override
    public void setSpeedSetpointRight(double setpointSpeedRPS) {
        mRightSetpoint = setpointSpeedRPS;
    }

    @Override
    public void setSpinDownMode(boolean spindownMode) {
        mSpinDownMode = spindownMode;
    }
}
