package frc.robot.subsystems.tilt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.poofsUtils.PoofsUtil;

import static frc.robot.subsystems.tilt.TiltConstants.*;

import java.util.function.Supplier;

public class Tilt extends SubsystemBase {
    public enum TiltGoalState {
        STOW(new TiltState(0.002, false, false)),
        CLOSE(new TiltState(0.07, false, true)),
        PODIUM(new TiltState(0.01, false, true)),
        MOONSHOT(new TiltState(0.055, false, true)),
        FEED(new TiltState(0.060, false, true)),
        AMP(new TiltState(0.078, false, true)),
        AUTO_AIM(new TiltState(0.017, true, true)),
        HAIL_MARY(new TiltState(0.005, false, true));

        public TiltState state;

        TiltGoalState(TiltState state) {
            this.state = state;
        }
    } 

    private TiltIO mIO;
    private TiltIOInputsAutoLogged mInputs;

    private TiltGoalState mGoalState = TiltGoalState.STOW;
    private boolean mWithinTolerance = false;

    private final LoggedTunableNumber kFeedforwardConstant = new LoggedTunableNumber("Tilt/Feedforward", kG);
    private double feedforward = kG;

    private final LoggedTunableBoolean kEnableTargetOverride = new LoggedTunableBoolean("Tilt/EnableOverride", false);
    private final LoggedTunableNumber kTargetOverride = new LoggedTunableNumber("Tilt/TargetOverride", TiltGoalState.STOW.state.defaultPosition);
    private boolean mEnableOverrideCache = false;
    private double mTargetOverrideCache = TiltGoalState.STOW.state.defaultPosition;

    public Tilt(TiltIO io) {
        mIO = io;
        mInputs = new TiltIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("ShooterTilt", mInputs);

        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = mInputs.tiltSuppliedCurrentAmps;
            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        // calculate robot-relative angle of shooter tilt TODO: set this minimum angle
        Rotation2d tiltAngle = Rotation2d.fromRotations(mInputs.tiltRotations).minus(Rotation2d.fromDegrees(20));

        kFeedforwardConstant.ifChanged(hashCode(), (kG) -> {this.feedforward = kG;});

        // calculate feedforward voltage for the current position
        double tiltFeedforward = getTiltFeedforward(tiltAngle);

        // cache goal state and record its values
        TiltGoalState goal = mGoalState;
        Logger.recordOutput("ShooterTilt/GoalState/BasePosition", goal.state.defaultPosition);
        Logger.recordOutput("ShooterTilt/GoalState/AutoAim", goal.state.autoAim);
        Logger.recordOutput("ShooterTilt/GoalState/StrictTolerance", goal.state.strictPositionTolerance);

        // get our tunable values from networktables
        kEnableTargetOverride.ifChanged(hashCode(), (v) -> mEnableOverrideCache = v);
        kTargetOverride.ifChanged(hashCode(), (v) -> mTargetOverrideCache = v);

        // calculate auto-aim angle if needed, and determine if within tolerance of target (also record outputs)
        double finalPositionTarget = goal.state.defaultPosition;
        if (mEnableOverrideCache) {
            finalPositionTarget = PoofsUtil.limit(mTargetOverrideCache, kMinTargetPosition, kMaxTargetPosition);
        } else if (goal.state.autoAim) {
            finalPositionTarget = RobotState.getInstance().getAimingParameters().tiltAngle().getRotations();
        }
        boolean withinTolerance = PoofsUtil.epsilonEquals(
            finalPositionTarget, 
            mInputs.tiltRotations, 
            goal.state.strictPositionTolerance ? kConservativeAllowableError : kLiberalAllowableError
        );
        mWithinTolerance = withinTolerance;
        Logger.recordOutput("ShooterTilt/FinalPositionTarget", finalPositionTarget);
        Logger.recordOutput("ShooterTilt/AutoAim", goal.state.autoAim);
        Logger.recordOutput("ShooterTilt/WithinTolerance", withinTolerance);

        // set targets and feedforwards
        mIO.setTiltTarget(finalPositionTarget);
        mIO.setTiltFeedForward(tiltFeedforward);
        mIO.updateOutputs();
    }

    public double getTiltFeedforward(Rotation2d angle) {
        return angle.plus(Rotation2d.fromDegrees(45)).getCos() * feedforward; //TODO: set feedforward
    }

    public void setGoalState(TiltGoalState goalState) {
        mGoalState = goalState;
        // assume that we are not within tolerance of the new target in the case that withinTolerance() is called before
        // the next run of periodic() (i.e. A command changes the goal state, then checks if the tilt has moved.)
        mWithinTolerance = false;
    }

    public Command setGoalCommand(TiltGoalState goalState) {
        return setGoalCommand(() -> goalState);
    }

    public Command setGoalCommand(Supplier<TiltGoalState> goalState) {
        return runOnce(() -> setGoalState(goalState.get())).andThen(
            Commands.waitUntil(this::withinTolerance)
        );
    }

    public boolean withinTolerance() {
        return mWithinTolerance;
    }

    public TiltGoalState getGoalState() {
        return mGoalState;
    }
}
