package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeSystemState;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;

public class Intake extends SubsystemBase {
    private IntakeIO mIO;
    private IntakeIOInputsAutoLogged mInputs;

    private final IntakeStateMachine mStateMachine;

    public Intake(IntakeIO io) {
        mIO = io;
        mInputs = new IntakeIOInputsAutoLogged();

        mStateMachine = new IntakeStateMachine();
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Intake", mInputs);

        if (Constants.getMode() == Mode.SIM) {
            Robot.updateSimCurrentDraw("Intake", mInputs.intakeSuppliedCurrentAmps);
        }

        double nextThrottle = mStateMachine.update(mInputs.intakeBeamBreakTriggered);

        Logger.recordOutput("Intake/WantedAction", mStateMachine.getWantedAction().name());
        Logger.recordOutput("Intake/SystemState", mStateMachine.getSystemState().name());
        Logger.recordOutput("Intake/DesiredThrottle", nextThrottle);

        mIO.setIntakeThrottle(nextThrottle);

        mIO.updateOutputs();
    }

    public void setWantedAction(IntakeWantedAction wantedAction) {
        mStateMachine.setWantedAction(wantedAction);
    }

    public Command setActionCommand(IntakeWantedAction wantedAction) {
        return setActionCommand(() -> wantedAction);
    }

    public Command setActionCommand(Supplier<IntakeWantedAction> wantedAction) {
        return runOnce(() -> setWantedAction(wantedAction.get()));
    }

    public Command setActionPersistCommand(IntakeWantedAction wantedAction) {
        return setActionPersistCommand(() -> wantedAction);
    }

    public Command setActionPersistCommand(Supplier<IntakeWantedAction> wantedAction) {
        return runOnce(() -> setWantedAction(wantedAction.get())).andThen(Commands.idle());
    }

    public Command waitUntilNoteCommand() {
        return Commands.waitUntil(this::hasNote);
    }

    public Command waitUntilEmptyCommand() {
        return Commands.waitUntil(() -> !this.hasNote());
    }

    public Command waitUntilSystemStateCommand(Supplier<IntakeSystemState> systemState) {
        return Commands.waitUntil(() -> mStateMachine.getSystemState().equals(systemState.get()));
    }

    public boolean hasNote() {
        return mInputs.intakeBeamBreakTriggered;
    }
}
