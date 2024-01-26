package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.indexer.IndexerStateMachine.SystemState;
import frc.robot.subsystems.indexer.IndexerStateMachine.WantedAction;

public class Indexer extends SubsystemBase{
    private IndexerIO mIO;
    private IndexerIOInputsAutoLogged mInputs;

    private IndexerStateMachine mStateMachine = new IndexerStateMachine();
    private WantedAction mWantedAction = WantedAction.OFF;

    public Indexer(IndexerIO io) {
        mIO = io;
        mInputs = new IndexerIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // Read inputs
        mIO.updateInputs(mInputs);
        Logger.processInputs("Indexer", mInputs);

        // Update sim stuff
        if (Constants.getMode() == Mode.SIM) {
            Robot.updateSimCurrentDraw("Indexer", mInputs.motorSuppliedCurrentAmps);
        }
        
        // Update state machine
        mStateMachine.setWantedAction(mWantedAction);
        double throttle = mStateMachine.update(mInputs.firstBannerTriggered, mInputs.secondBannerTriggered);

        // Send RealOutputs
        Logger.recordOutput("Indexer/WantedAction", mWantedAction.name());
        Logger.recordOutput("Indexer/SystemState", mStateMachine.getSystemState().name());
        Logger.recordOutput("Indexer/DesiredThrottle", throttle);

        // Send outputs to motors
        mIO.setMotorThrottle(throttle);
        mIO.updateOutputs();
    }

    public void setWantedAction(WantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
        }
    }

    public SystemState getSystemState() {
        return mStateMachine.getSystemState();
    }
}
