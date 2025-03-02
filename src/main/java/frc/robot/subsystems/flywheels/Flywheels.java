package frc.robot.subsystems.flywheels;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsSystemState;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;

public class Flywheels extends SubsystemBase{
    private FlywheelsIO mIO;
    private FlywheelsIOInputsAutoLogged mInputs;

    private double mSetpointSpeedLeft = 0.0; //18, 65
    private double mSetpointSpeedRight = 0.0; //22, 55
    private FlywheelsStateMachine mStateMachine = new FlywheelsStateMachine();
    private FlywheelsWantedAction mWantedAction = FlywheelsWantedAction.OFF;

    public Flywheels(FlywheelsIO io) {
        mIO = io;
        mInputs = new FlywheelsIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // read inputs
        mIO.updateInputs(mInputs);
        Logger.processInputs("Shooter", mInputs);

        // update sim stuff
        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            simCurrent += mInputs.leftMotorSuppliedCurrentAmps;
            simCurrent += mInputs.rightMotorSpeedRPS;
            Robot.updateSimCurrentDraw("Shooter", simCurrent);
        }

        // update state machine
        mStateMachine.setWantedAction(mWantedAction);
        FlywheelsState outputState = mStateMachine.update(mInputs.leftMotorSpeedRPS, mInputs.rightMotorSpeedRPS, mSetpointSpeedLeft, mSetpointSpeedRight);

        double finalSetpointLeft = mSetpointSpeedLeft;
        double finalSetpointRight = mSetpointSpeedRight;
        if (outputState.getSlowIdle()) {
            finalSetpointLeft = FlywheelsConstants.kIdleRPS;
            finalSetpointRight = FlywheelsConstants.kIdleRPS;
        }

        // send realoutputs
        Logger.recordOutput("Shooter/WantedAction", mWantedAction);
        Logger.recordOutput("Shooter/SystemState", mStateMachine.getSystemState());
        Logger.recordOutput("Shooter/StoredSetpoint/LeftRPS", mSetpointSpeedLeft);
        Logger.recordOutput("Shooter/StoredSetpoint/RightRPS", mSetpointSpeedRight);
        Logger.recordOutput("Shooter/FinalSetpoint/LeftRPS", finalSetpointLeft);
        Logger.recordOutput("Shooter/FinalSetpoint/RightRPS", finalSetpointRight);
        Logger.recordOutput("Shooter/EnableFlywheels", outputState.getFlywheelEnabled());
        Logger.recordOutput("Shooter/SlowIdle", outputState.getSlowIdle());
        Logger.recordOutput("Shooter/BrakeMode", outputState.getBrakeModeEnabled());

        // send outputs to motors
        mIO.setSpeedSetpointLeft(finalSetpointLeft);
        mIO.setSpeedSetpointRight(finalSetpointRight);
        mIO.setBrakeMode(outputState.getBrakeModeEnabled());
        mIO.setSpinDownMode(!outputState.getFlywheelEnabled());

        mIO.updateOutputs();
    }

    public void setWantedAction(FlywheelsWantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
        }
    }

    public Command setActionCommand(FlywheelsWantedAction action) {
        return setActionCommand(() -> action);
    }

    public Command setActionCommand(Supplier<FlywheelsWantedAction> action) {
        return runOnce(() -> setWantedAction(action.get()));
    }

    public Command setActionPersistCommand(FlywheelsWantedAction action) {
        return setActionPersistCommand(() -> action);
    }

    public Command setActionPersistCommand(Supplier<FlywheelsWantedAction> action) {
        return runOnce(() -> setWantedAction(action.get())).andThen(Commands.idle());
    }

    public Command waitUntilStateCommand(FlywheelsSystemState state) {
        return waitUntilStateCommand(() -> state);
    }

    public Command waitUntilStateCommand(Supplier<FlywheelsSystemState> state) {
        return Commands.waitUntil(() -> mStateMachine.getSystemState().equals(state.get()));
    }

    public Command setSpeedCommand(Supplier<Double> speed) {
        return runOnce(() -> {
            setSetpointSpeedBottom(speed.get());
            setSetpointSpeedTop(speed.get());
        } );
    }

    public void setSetpointSpeedTop(double setpointRPS) {
        mSetpointSpeedLeft = setpointRPS;
    }

    public void setSetpointSpeedBottom(double setpointRPS) {
        mSetpointSpeedRight = setpointRPS;
    }

    public FlywheelsStateMachine.FlywheelsSystemState getSystemState() {
        return mStateMachine.getSystemState();
    }
}
