package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.lib.dashboard.DashboardToggleSwitch;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim mMotorSim;

    private double mSetpoint = 0.0;

    private final DashboardToggleSwitch mBanner1Triggered;
    private final DashboardToggleSwitch mBanner2Triggered;

    public IndexerIOSim() {
        mMotorSim = new FlywheelSim(DCMotor.getKrakenX60(1), 9.19, 0.0025);

        mBanner1Triggered = new DashboardToggleSwitch("Banner1SimTrigger", false);
        mBanner2Triggered = new DashboardToggleSwitch("Banner2SimTrigger", false);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorSpeedRPS = mMotorSim.getAngularVelocityRPM() / 60.0;
        inputs.motorSuppliedCurrentAmps = mMotorSim.getCurrentDrawAmps();
        inputs.motorTempCelsius = 0;
        inputs.firstBannerTriggered = mBanner1Triggered.getAsBoolean();
        inputs.secondBannerTriggered = mBanner2Triggered.getAsBoolean();
    }

    @Override
    public void setMotorThrottle(double throttle) {
        mSetpoint = throttle;
    }

    @Override
    public void updateOutputs() {
        mMotorSim.setInputVoltage(12.0 * mSetpoint);
        mMotorSim.update(Constants.loopPeriodSecs);
    }
}
