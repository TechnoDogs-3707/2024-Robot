package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import frc.robot.Constants;
import frc.robot.Constants.ArmSubsystem.J1;
import frc.robot.Constants.ArmSubsystem.J2;

public class ArmIOSimV1 implements ArmIO {
    private final TrapezoidProfile.Constraints kJ1Constraints;
    private final TrapezoidProfile.Constraints kJ2Constraints;

    private TrapezoidProfile.State mJ1LastState;
    private TrapezoidProfile.State mJ2LastState;

    private TrapezoidProfile.State mJ1GoalState;
    private TrapezoidProfile.State mJ2GoalState;

    private TrapezoidProfile mJ1Estimator;
    private TrapezoidProfile mJ2Estimator;

    public ArmIOSimV1() {
        kJ1Constraints = new Constraints(J1.kMagicVel, J1.kMagicAccel);
        kJ2Constraints = new Constraints(J2.kMagicVel, J2.kMagicAccel);

        mJ1LastState = new State(J1.kHomePosition, 0);
        mJ2LastState = new State(J2.kHomePosition, 0);

        mJ1GoalState = new State(J1.kHomePosition, 0);
        mJ2GoalState = new State(J2.kHomePosition, 0);

        mJ1Estimator = new TrapezoidProfile(kJ1Constraints);
        mJ2Estimator = new TrapezoidProfile(kJ2Constraints);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.tiltRotations = mJ1LastState.position;
        inputs.tiltVelocityRotPerSec = mJ1LastState.velocity;
        
        inputs.wristRotations = mJ2LastState.position;
        inputs.wristVelocityRotPerSec = mJ2LastState.velocity;
    }

    @Override
    public void updateOutputs() {
        mJ1LastState = mJ1Estimator.calculate(Constants.loopPeriodSecs, mJ1LastState, mJ1GoalState);
        mJ2LastState = mJ2Estimator.calculate(Constants.loopPeriodSecs, mJ2LastState, mJ2GoalState);
    }

    @Override
    public void setTiltTarget(double rotations) {
        mJ1GoalState.position = rotations;
    }

    @Override
    public void setWristTarget(double rotations) {
        mJ2GoalState.position = rotations;
    }
}