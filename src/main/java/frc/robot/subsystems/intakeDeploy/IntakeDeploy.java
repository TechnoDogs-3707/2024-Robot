package frc.robot.subsystems.intakeDeploy;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.dashboard.LoggedTunableNumber;

public class IntakeDeploy extends SubsystemBase {
    public enum IntakePositionPreset {
        STOWED(0.3),
        EJECT(0.1875),
        DEPLOYED(-0.05);

        public double position;

        private IntakePositionPreset(double position) {
            this.position = position;
        }
    }

    private IntakeDeployIO mIO;
    private IntakeDeployIOInputsAutoLogged mInputs;

    private IntakePositionPreset mPositionPreset = IntakePositionPreset.STOWED;
    private boolean mUseManualThrottle = false;
    private double mManualThrottle = 0.0;

    private boolean mWithinTolerance = false;

    //TODO: move this to the IO TalonFX class.
    private final LoggedTunableNumber kFeedforwardConstant = new LoggedTunableNumber("IntakeDeploy/Feedforward", IntakeDeployConstants.kG);
    private double feedforward = IntakeDeployConstants.kG;

    public IntakeDeploy(IntakeDeployIO io) {
        mIO = io;
        mInputs = new IntakeDeployIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // Update and process inputs

        // update sim current draw

        // cache goal state and current limit, then record outputs

        // set targets and update outputs
    }

    public void setPositionPreset(IntakePositionPreset positionPreset) {
        mPositionPreset = positionPreset;
        mWithinTolerance = false;
    }

    public boolean atTarget() {
        return mWithinTolerance;
    }

    public Command setPositionBlockingCommand(IntakePositionPreset preset) {
        return setPositionBlockingCommand(() -> preset);
    }

    public Command setPositionBlockingCommand(Supplier<IntakePositionPreset> preset) {
        return runOnce(() -> setPositionBlockingCommand(preset.get())).andThen(
            Commands.waitUntil(this::atTarget)
        );
    }

    public Command setPositionCommand(IntakePositionPreset preset) {
        return setPositionCommand(() -> preset);
    }

    public Command setPositionCommand(Supplier<IntakePositionPreset> preset) {
        return runOnce(() -> setPositionBlockingCommand(preset.get()));
    }
}
