// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.lib.phoenixpro.PhoenixErrorChecker;

/** Add your docs here. */
public class GyroPigeonIO implements GyroIO {
    private final Pigeon2 gyro;
    private final Pigeon2Configuration gyroConfig;

    private final StatusSignal<Double> yawSignal;
    private final StatusSignal<Double> pitchSignal;
    private final StatusSignal<Double> rollSignal;

    private final StatusSignal<Double> yawVelocitySignal;
    private final StatusSignal<Double> pitchVelocitySignal;
    private final StatusSignal<Double> rollVelocitySignal;

    private final StatusSignal<Double> quatWSignal;
    private final StatusSignal<Double> quatXSignal;
    private final StatusSignal<Double> quatYSignal;
    private final StatusSignal<Double> quatZSignal;

    private final ArrayList<StatusSignal<?>> statusSignals;

    public GyroPigeonIO(int id, String bus) {
        this.gyro = new Pigeon2(id, bus);
        gyroConfig = new Pigeon2Configuration();

        PhoenixErrorChecker.checkErrorAndRetry(() -> gyro.getConfigurator().apply(gyroConfig));
        PhoenixErrorChecker.checkErrorAndRetry(() -> gyro.setYaw(0.0));

        yawSignal = gyro.getYaw();
        pitchSignal = gyro.getPitch();
        rollSignal = gyro.getRoll();

        yawVelocitySignal = gyro.getAngularVelocityZDevice();
        pitchVelocitySignal = gyro.getAngularVelocityYDevice();
        rollVelocitySignal = gyro.getAngularVelocityXDevice();

        quatWSignal = gyro.getQuatW();
        quatXSignal = gyro.getQuatX();
        quatYSignal = gyro.getQuatY();
        quatZSignal = gyro.getQuatZ();

        statusSignals = new ArrayList<>();

        statusSignals.add(yawSignal);
        statusSignals.add(pitchSignal);
        statusSignals.add(rollSignal);
        statusSignals.add(yawVelocitySignal);
        statusSignals.add(pitchVelocitySignal);
        statusSignals.add(rollVelocitySignal);
        statusSignals.add(quatWSignal);
        statusSignals.add(quatXSignal);
        statusSignals.add(quatYSignal);
        statusSignals.add(quatZSignal);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        statusSignals.forEach((v) -> v.refresh());

        inputs.calibrating = false; // pigeon2 does not calibrate during normal operation, so we can ignore this flag.
        inputs.connected = !yawSignal.getStatus().isError(); // if the yaw is errored, we should probably ignore the readings

        inputs.yawAngleRotations = yawSignal.getValue() / 360.0;
        inputs.pitchAngleRotations = pitchSignal.getValue() / 360.0;
        inputs.rollAngleRotations = rollSignal.getValue() / 360.0;

        inputs.yawVelocityRotationsPerSecond = yawVelocitySignal.getValue() / 360.0;
        inputs.pitchVelocityRotationsPerSecond = pitchVelocitySignal.getValue() / 360.0;
        inputs.rollVelocityRotationsPerSecond = rollVelocitySignal.getValue() / 360.0;

        inputs.quaternionW = quatWSignal.getValue();
        inputs.quaternionX = quatXSignal.getValue();
        inputs.quaternionY = quatYSignal.getValue();
        inputs.quaternionZ = quatZSignal.getValue();
    }
}
