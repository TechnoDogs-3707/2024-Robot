package frc.robot.subsystems.shooterFlywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.WantedAction;

public class ShooterFlywheels extends SubsystemBase{
    private ShooterFlywheelsIO mIO;
    private ShooterIOInputsAutoLogged mInputs;

    private double mSetpointSpeedLeft = 0.0;
    private double mSetpointSpeedRight = 0.0;
    private ShooterFlywheelsStateMachine mStateMachine = new ShooterFlywheelsStateMachine();
    private WantedAction mWantedAction = WantedAction.OFF;

    public ShooterFlywheels(ShooterFlywheelsIO io) {
        mIO = io;
        mInputs = new ShooterIOInputsAutoLogged();
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
        ShooterFlywheelsState outputState = mStateMachine.update(mInputs.leftMotorSpeedRPS, mInputs.rightMotorSpeedRPS, mSetpointSpeedLeft, mSetpointSpeedRight);

        double finalSetpointLeft = mSetpointSpeedLeft;
        double finalSetpointRight = mSetpointSpeedRight;
        if (outputState.getSlowIdle()) {
            finalSetpointLeft = Constants.Shooter.kIdleRPS;
            finalSetpointRight = Constants.Shooter.kIdleRPS;
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
    }

    public void setWantedAction(WantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
        }
    }

    public void setSetpointSpeedLeft(double setpointRPS) {
        mSetpointSpeedLeft = setpointRPS;
    }

    public void setSetpointSpeedRight(double setpointRPS) {
        mSetpointSpeedRight = setpointRPS;
    }
}
