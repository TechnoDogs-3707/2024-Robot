package frc.robot.subsystems.shooterTilt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.lib.dashboard.LoggedTunableNumber;
import frc.robot.lib.util.Util;

import static frc.robot.Constants.ShooterTilt.*;

public class ShooterTilt extends SubsystemBase {
    public enum ShooterTiltGoalState {
        STOW(new ShooterTiltState(0.01, false, false)),
        CLOSE(new ShooterTiltState(0.10, false, true)),
        PODIUM(new ShooterTiltState(0.05, false, true)),
        AUTO_AIM(new ShooterTiltState(0.03, true, true));

        public ShooterTiltState state;

        ShooterTiltGoalState(ShooterTiltState state) {
            this.state = state;
        }
    } 

    private ShooterTiltIO mIO;
    private ShooterTiltIOInputsAutoLogged mInputs;

    private ShooterTiltGoalState mGoalState = ShooterTiltGoalState.STOW;
    private boolean mWithinTolerance = false;

    private final LoggedTunableNumber kFeedforwardConstant = new LoggedTunableNumber("Tilt/Feedforward", kG);
    private double feedforward = kG;

    public ShooterTilt(ShooterTiltIO io) {
        mIO = io;
        mInputs = new ShooterTiltIOInputsAutoLogged();
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
        ShooterTiltGoalState goal = mGoalState;
        Logger.recordOutput("ShooterTilt/GoalState/BasePosition", goal.state.defaultPosition);
        Logger.recordOutput("ShooterTilt/GoalState/AutoAim", goal.state.autoAim);
        Logger.recordOutput("ShooterTilt/GoalState/StrictTolerance", goal.state.strictPositionTolerance);

        // calculate auto-aim angle if needed, and determine if within tolerance of target (also record outputs)
        double finalPositionTarget = goal.state.defaultPosition;
        if (goal.state.autoAim) {
            finalPositionTarget = 0.0; // TODO: auto-aim lookup table or formula
        }
        boolean withinTolerance = Util.epsilonEquals(
            goal.state.defaultPosition, 
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
        return angle.plus(Rotation2d.fromDegrees(20.8)).getCos() * feedforward; //TODO: set feedforward
    }

    public void setGoalState(ShooterTiltGoalState goalState) {
        mGoalState = goalState;
        // assume that we are not within tolerance of the new target in the case that withinTolerance() is called before
        // the next run of periodic() (i.e. A command changes the goal state, then checks if the tilt has moved.)
        mWithinTolerance = false;
    }

    public boolean withinTolerance() {
        return mWithinTolerance;
    }

    public ShooterTiltGoalState getGoalState() {
        return mGoalState;
    }
}
