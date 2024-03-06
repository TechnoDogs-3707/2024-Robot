package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateTracker;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ArmSubsystem.J2;
import frc.robot.lib.dashboard.LoggedTunableNumber;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.lib.drive.AutoAlignPointSelector.RequestedAlignment;
import frc.robot.subsystems.arm.ArmState.ArmSend;

public class Arm extends SubsystemBase {

    public enum GoalState {
        STOW(ArmState.withConservativeConstraints(0, 0.3, ArmSend.LOW)),
        INTAKE_GROUND(ArmState.withConservativeConstraints(0, -0.1, ArmSend.LOW)),
        INTAKE_SOURCE(ArmState.withConservativeConstraints(0.24, 0, ArmSend.LOW)),
        SCORE_AMP(ArmState.withConservativeConstraints(0.27, 0.0, ArmSend.LOW)),
        HANDOFF(ArmState.withConservativeConstraints(0, -0.1, ArmSend.LOW));
        
        public ArmState state;

        GoalState(ArmState goalState) {
            this.state = goalState;
        }
    }

    private final ArmMotionPlanner mMotionPlanner;

    private ArmState mMeasuredState = new ArmState();
    private ArmState mCommandedState = new ArmState();
    private ArmState mLastCommandedState = new ArmState();
    private GoalState mGoalState = GoalState.STOW;
    private GoalState mLastGoalState = GoalState.STOW;
    private boolean mResetMotionPlanner = false;

    private double wristKf = J2.kG;
    private final LoggedTunableNumber mTunableWristFeedforward = new LoggedTunableNumber("Arm/Wrist/Feedforward", wristKf);

    // TODO: Update Feedforwards and constants
    public static final double kJ1BaseLength = Units.inchesToMeters(19); // distance from J1 pivot to J2 pivot
    public static final double kJ1ForceGravitySelfVolts = 0.0; // Volts required to hold J1 horizontal and J2 vertical
    public static final double kJ1ForceGravityJ2Volts = 0.0; // Volts to add to J1 when J2 is at full extension
    public static final double kJ2ForceGravitySelfVolts = 0.0; // Volts to hold J2 in horizontal position

    private final Mechanism2d mSensorMech = new Mechanism2d(1.75, 1.75);
    private final MechanismRoot2d mSensorMechRoot = mSensorMech.getRoot("Base", 0.25, 0.25);
    private final MechanismLigament2d mSensorMechJ1 = mSensorMechRoot
            .append(new MechanismLigament2d("J1", kJ1BaseLength, 0, 5, new Color8Bit(Color.kOrange)));
    private final MechanismLigament2d mSensorMechJ2 = mSensorMechJ1
            .append(new MechanismLigament2d("J2", Units.inchesToMeters(12), 235, 5, new Color8Bit(Color.kPurple)));

    private final Mechanism2d mTargetMech = new Mechanism2d(1.75, 1.75);
    private final MechanismRoot2d mTargetMechRoot = mTargetMech.getRoot("Base", 0.25, 0.25);
    private final MechanismLigament2d mTargetMechJ1 = mTargetMechRoot
            .append(new MechanismLigament2d("J1", kJ1BaseLength, 0, 5, new Color8Bit(Color.kLightGray)));
    private final MechanismLigament2d mTargetMechJ2 = mTargetMechJ1
            .append(new MechanismLigament2d("J2", Units.inchesToMeters(12), 235, 5, new Color8Bit(Color.kLightGray)));

    private ArmIO mArmIO;
    private ArmIOInputsAutoLogged mArmInputs;

    public Arm(ArmIO armIO) {
        mMotionPlanner = new ArmMotionPlanner();

        mArmIO = armIO;
        mArmInputs = new ArmIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        mArmIO.updateInputs(mArmInputs);
        Logger.processInputs("Arm", mArmInputs);

        // double timestamp = Timer.getFPGATimestamp();

        //TODO: led states

        // Optional<TimedLEDState> ledState = handleLEDs(timestamp);
        // if (DriverStation.isEnabled()) {
        //     if (ledState.isPresent()) {
        //         if (DriverStation.isAutonomous()) {
        //             LED.setWantedAction(LED.WantedAction.DISPLAY_VISION);
        //         } else {
        //             // LED.setDeliveryLEDState(ledState.get()); 
        //             LED.setWantedAction(LED.WantedAction.DISPLAY_ARM);
        //         }
        //     } else {
        //         LED.setWantedAction(LED.WantedAction.OFF);
        //     }
        // }
        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            simCurrent += mArmInputs.tiltSuppliedCurrentAmps;
            simCurrent += mArmInputs.wristSuppliedCurrentAmps;

            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        Rotation2d tiltAngle = Rotation2d.fromRotations(mArmInputs.tiltRotations);
        Rotation2d wristAngle = Rotation2d.fromRotations(mArmInputs.wristRotations);

        mSensorMechJ1.setAngle(tiltAngle);
        mSensorMechJ2.setAngle(wristAngle.plus(tiltAngle.unaryMinus()));

        mMeasuredState = new ArmState(mArmInputs.tiltRotations, mArmInputs.wristRotations, mCommandedState.send);

        ArmState nextArmState = mMotionPlanner.update(mMeasuredState);
        mLastCommandedState = mCommandedState;
        mCommandedState = nextArmState;

        Logger.recordOutput("Arm/GoalState/Name", mGoalState.name());
        Logger.recordOutput("Arm/RequestedAlignment", getRequestedAlignment().name());
        Logger.recordOutput("Arm/GoalState/Send", mGoalState.state.send.name());
        Logger.recordOutput("Arm/CommandedState/Tolerance/J1", mCommandedState.j1Tolerance);
        Logger.recordOutput("Arm/CommandedState/Tolerance/J2", mCommandedState.j2Tolerance);

        mTargetMechJ1.setAngle(Rotation2d.fromRotations(mCommandedState.j1));
        mTargetMechJ2.setAngle(Rotation2d.fromRotations(mCommandedState.j2).plus(Rotation2d.fromRotations(mCommandedState.j1).unaryMinus()));

        Logger.recordOutput("Arm/MeasuredPositions", mSensorMech);
        Logger.recordOutput("Arm/TargetPositions", mTargetMech);
        Logger.recordOutput("Arm/MotionPlanner/AtGoal", atGoal());
        Logger.recordOutput("Arm/MotionPlanner/StatesRemaining", mMotionPlanner.getRemainingStates());

        var currentPose = RobotStateTracker.getInstance().getCurrentRobotPose();
        RobotStateTracker.getInstance().setAutoAlignReady(AutoAlignPointSelector.getAlignTarget(currentPose, getRequestedAlignment()).isPresent());

        mTunableWristFeedforward.ifChanged(hashCode(), (v) -> wristKf = v);

        if (mResetMotionPlanner) {
            mMotionPlanner.reset();
        }

        if (mCommandedState.j1 != mLastCommandedState.j1) {
            mArmIO.setTiltTarget(mCommandedState.j1);
        }
        double j1FeedForward = calcJ1Feedforward();
        Logger.recordOutput("Arm/CalculatedFeedForwards/J1", j1FeedForward);
        mArmIO.setTiltFeedForward(j1FeedForward);

        if (mCommandedState.j2 != mLastCommandedState.j2 ) {
            mArmIO.setWristTarget(mCommandedState.j2);
        }
        double j2FeedForward = calcJ2FeedForward();
        Logger.recordOutput("Arm/CalculatedFeedForwards/J2", j2FeedForward);
        mArmIO.setWristFeedForward(j2FeedForward);

        mArmIO.updateOutputs();
    }

    public ArmState getMeasuredState() {
        return mMeasuredState;
    }

    public boolean getResetMotionPlanner() {
        return mResetMotionPlanner;
    }

    public void setResetMotionPlanner(boolean resetMotionPlanner) {
        mResetMotionPlanner = resetMotionPlanner;
    } 

    public boolean atGoal() {
        return mMotionPlanner.isFinished() && mMeasuredState.isInRange(mCommandedState);
    }

    public GoalState getGoalState() {
        return mGoalState;
    }

    public GoalState getLastGoalState() {
        return mLastGoalState;
    }

    public void setGoalState(GoalState goalState) {
        mLastGoalState = mGoalState;
        mGoalState = goalState;

        mMotionPlanner.setDesiredState(mGoalState.state, mMeasuredState);
    }

    public Command setGoalCommand(GoalState goal) {
        return setGoalCommand(() -> goal);
    }

    public Command setGoalCommand(Supplier<GoalState> goal) {
        return runOnce(() -> setGoalState(goal.get()))
            .andThen(Commands.waitUntil(this::atGoal));
    }

    private double calcJ1Feedforward() {
        return 0; // TODO: J1 feedforward
    }

    private double calcJ2FeedForward() {
        return Rotation2d.fromDegrees(mArmInputs.wristRotations).getCos() * wristKf;
    }

    public RequestedAlignment getRequestedAlignment() {
        return RequestedAlignment.AUTO; // TODO: calculate requested alignment
    }
}
