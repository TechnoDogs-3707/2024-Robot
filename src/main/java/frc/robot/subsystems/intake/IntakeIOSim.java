package frc.robot.subsystems.intake;

import frc.robot.lib.dashboard.DashboardToggleSwitch;

public class IntakeIOSim implements IntakeIO {
    
    private final DashboardToggleSwitch mContrastSensor;

    public IntakeIOSim() {
        mContrastSensor = new DashboardToggleSwitch("IntakeContrastSensor", false);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeBeamBreakTriggered = mContrastSensor.getAsBoolean();
    }
}
