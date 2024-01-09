// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SimSwerveIO implements SwerveModuleIO {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.05);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.04);

    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 0.75);
    private PIDController steerPID = new PIDController(48, 0, 0);

    private double turnAbsolutePosition = Math.random();
    private double turnAppliedVolts = 0.0;

    public SimSwerveIO() {
        System.out.println("[Init] Creating ModuleIOSim");
        steerPID.enableContinuousInput(0, 1);
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);

        double angleDiff = turnSim.getAngularVelocityRPM() / 60.0 * Constants.loopPeriodSecs;
        turnAbsolutePosition += angleDiff;
        while (turnAbsolutePosition < 0.0) {
            turnAbsolutePosition += 1.0;
        }
        while (turnAbsolutePosition > 1.0) {
            turnAbsolutePosition -= 1.0;
        }

        inputs.driveMeters = inputs.driveMeters
                + convertRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0 * Constants.loopPeriodSecs);
        inputs.driveVelocityMetersPerSec = convertRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0);
        inputs.driveSuppliedCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelsius = -1.0;

        inputs.steerPositionRotations = turnAbsolutePosition;
        inputs.steerVelocityRotPerSec = turnSim.getAngularVelocityRPM() / 60.0;
        inputs.steerSuppliedCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        inputs.steerTempCelsius = -1.0;
    }

    public void setDriveVoltage(double volts) {
        driveSim.setInputVoltage(volts);
    }

    @Override
    public void setDriveSpeedClosedLoop(double speedMetersPerSecond) {
        setDriveThrottleOpenLoop(speedMetersPerSecond);
    }

    @Override
    public void setDriveThrottleOpenLoop(double speedMetersPerSecond) {
        double speedTargetRPS = convertMetersToRotations(speedMetersPerSecond);
        setDriveVoltage(driveFF.calculate(speedTargetRPS));
    }

    @Override
    public void setSteerPositionTarget(double steerAngleRotations) {
        setTurnVoltage(steerPID.calculate(turnAbsolutePosition, steerAngleRotations));
    }

    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    private double convertRotationsToMeters(double rotations) {
        double wheelCircumference = Constants.kDriveWheelDiameter * Math.PI;
        double metersPerMotorRotation = wheelCircumference / Constants.kDriveReduction;

        return rotations * metersPerMotorRotation;
    }

    private double convertMetersToRotations(double meters) {
        double wheelCircumference = Constants.kDriveWheelDiameter * Math.PI;
        double motorRotationsPerMeter = Constants.kDriveReduction / wheelCircumference;

        return meters * motorRotationsPerMeter;
    }

    @Override
    public void stop() {
        setDriveThrottleOpenLoop(0);
    }
}