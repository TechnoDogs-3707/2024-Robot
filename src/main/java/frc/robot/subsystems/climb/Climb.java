package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.lib.util.Util;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import java.util.function.Supplier;

public class Climb extends SubsystemBase {
    private ClimbIO mIO;
    private ClimbIOInputsAutoLogged mInputs;

    public enum ClimbMode {
        MANUAL_THROTTLE,
        PID_EXTEND,
        PID_RETRACT,
        CLIMB_RETRACT
    }

    private ClimbMode mMode = ClimbMode.MANUAL_THROTTLE;

    private double mManualThrottle = 0.0;

    public Climb(ClimbIO io) {
        mIO = io;
        mInputs = new ClimbIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Climb", mInputs);

        //update current draw simluation
        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            simCurrent += mInputs.leftMotorSuppliedCurrentAmps;
            simCurrent += mInputs.rightMotorSuppliedCurrentAmps;
            Robot.updateSimCurrentDraw("Climb", simCurrent);
        }

        double positionTarget = 0.0;
        double overrideThrottle = 0.0;
        boolean usePID = false;

        //update state machine
        switch (mMode) {
            case CLIMB_RETRACT:
                positionTarget = 0.0;
                overrideThrottle = -1.0;
                usePID = false;
                break;
            case MANUAL_THROTTLE:
                positionTarget = 0.0;
                overrideThrottle = mManualThrottle;
                usePID = false;
                break;
            case PID_EXTEND:
                positionTarget = kFullExtensionPosition;
                overrideThrottle = 0.0;
                usePID = true;
                break;
            case PID_RETRACT:
                positionTarget = kFullRetractionPosition;
                overrideThrottle = 0.0;
                usePID = true;
                break;
            default:
                break;
        }

        //record outputs
        Logger.recordOutput("Climb/ModeName", mMode.name());
        Logger.recordOutput("Climb/UsePID", usePID);
        Logger.recordOutput("Climb/OverrideThrottle", overrideThrottle);
        Logger.recordOutput("Climb/PositionTarget", positionTarget);

        //send outputs to motors
        mIO.enablePID(usePID);
        mIO.setThrottle(overrideThrottle);
        mIO.setPositionTarget(positionTarget);

        mIO.updateOutputs();
    }

    public void setManualThrottle(double throttle) {
        mManualThrottle = throttle;
    }

    public void setClimbMode(ClimbMode mode) {
        mMode = mode;
    }

    public Command setModeCommand(Supplier<ClimbMode> mode) {
        return runOnce(() -> setClimbMode(mode.get()));
    }
    
    public Command setModeCommand(ClimbMode mode) {
        return setModeCommand(() -> mode);
    }

    public boolean isAtPosition(double position) {
        return Util.epsilonEquals(mInputs.leftMotorRotations, position, kPIDAllowableError) && Util.epsilonEquals(mInputs.rightMotorRotations, position, kPIDAllowableError);
    }

    public Command setThrottleCommand(Supplier<Double> throttle) {
        return setModeCommand(ClimbMode.MANUAL_THROTTLE).andThen(run(() -> setManualThrottle(throttle.get())));
    }
}
