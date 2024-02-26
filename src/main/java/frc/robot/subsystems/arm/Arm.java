package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateTracker;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.lib.drive.AutoAlignPointSelector.RequestedAlignment;
import frc.robot.lib.leds.LEDState;
import frc.robot.lib.leds.TimedLEDState;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.arm.ArmState.ArmSend;
import frc.robot.subsystems.leds.LED;

public class Arm extends SubsystemBase {

    public enum GoalState {
        STOW(ArmState.withConservativeConstraints(0, 0, ArmSend.LOW)),
        INTAKE_GROUND(ArmState.withConservativeConstraints(0, 0.5, ArmSend.LOW)),
        INTAKE_SOURCE(ArmState.withConservativeConstraints(0.24, 0, ArmSend.LOW)),
        SCORE_AMP(ArmState.withConservativeConstraints(0.27, 0.0, ArmSend.LOW)),
        SCORE_SPEAKER_SUBWOOFER(ArmState.withConservativeConstraints(0, 0, ArmSend.LOW));
        
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

    private ArmIntakeStateMachine mArmIntakeStateMachine = new ArmIntakeStateMachine();

    private boolean mForceFailure = false;

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

        double timestamp = Timer.getFPGATimestamp();

        Optional<TimedLEDState> ledState = handleLEDs(timestamp);
        if (DriverStation.isEnabled()) {
            if (ledState.isPresent()) {
                if (DriverStation.isAutonomous()) {
                    LED.setWantedAction(LED.WantedAction.DISPLAY_VISION);
                } else {
                    LED.setArmLEDState(ledState.get());
                    LED.setWantedAction(LED.WantedAction.DISPLAY_ARM);
                }
            } else {
                LED.setWantedAction(LED.WantedAction.OFF);
            }
        }
        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            simCurrent += mArmInputs.tiltSuppliedCurrentAmps;
            simCurrent += mArmInputs.wristSuppliedCurrentAmps;
            simCurrent += mArmInputs.intakeSuppliedCurrentAmps;

            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        Rotation2d tiltAngle = Rotation2d.fromRotations(mArmInputs.tiltRotations);
        Rotation2d wristAngle = Rotation2d.fromRotations(mArmInputs.wristRotations);

        mSensorMechJ1.setAngle(tiltAngle);
        mSensorMechJ2.setAngle(wristAngle.plus(tiltAngle).unaryMinus().minus(Rotation2d.fromDegrees(235)));

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
        mTargetMechJ2.setAngle(Rotation2d.fromRotations(mCommandedState.j2).plus(Rotation2d.fromRotations(mCommandedState.j1)).unaryMinus().minus(Rotation2d.fromDegrees(235)));

        Logger.recordOutput("Arm/MeasuredPositions", mSensorMech);
        Logger.recordOutput("Arm/TargetPositions", mTargetMech);
        Logger.recordOutput("Arm/MotionPlanner/AtGoal", atGoal());
        Logger.recordOutput("Arm/MotionPlanner/StatesRemaining", mMotionPlanner.getRemainingStates());

        var currentPose = RobotStateTracker.getInstance().getCurrentRobotPose();
        RobotStateTracker.getInstance().setAutoAlignReady(AutoAlignPointSelector.getAlignTarget(currentPose, getRequestedAlignment()).isPresent());

        if (mResetMotionPlanner) {
            mMotionPlanner.reset();
        }

        boolean intakeBeamBreakTriggered = mArmInputs.intakeBeamBreakTriggered;
        double intakeThrottle = mArmIntakeStateMachine.update(intakeBeamBreakTriggered);
        Logger.recordOutput("Arm/Intake/BeamBreakTriggered", intakeBeamBreakTriggered);
        Logger.recordOutput("Arm/Intake/StateMachine/WantedAction", mArmIntakeStateMachine.getWantedAction());
        Logger.recordOutput("Arm/Intake/StateMachine/SystemState", mArmIntakeStateMachine.getSystemState());
        Logger.recordOutput("Arm/Intake/StateMachine/Throttle", intakeThrottle);
        mArmIO.setIntakeThrottle(intakeThrottle);

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

    public void setForceFailure(boolean forceFailure) {
        mForceFailure = forceFailure;
    }

    private Rotation2d calcJ2RefAngle() {
        return Rotation2d.fromRotations(mArmInputs.wristRotations).minus(Rotation2d.fromRotations(mArmInputs.tiltRotations));
    }

    private double calcJ1Feedforward() {
        return 0; // TODO: J1 feedforward
    }

    private double calcJ2FeedForward() {
        return 0; // TODO: J2 feedforward
    }

    public void setWantedAction(ArmIntakeStateMachine.ArmIntakeWantedAction wantedAction) {
        mArmIntakeStateMachine.setWantedAction(wantedAction);
    }

    public ArmIntakeStateMachine.ArmIntakeSystemState getIntakeSystemState() {
        return mArmIntakeStateMachine.getSystemState();
    }

    public boolean intakeHasGamepiece() {
        return mArmInputs.intakeBeamBreakTriggered;
    }

    public RequestedAlignment getRequestedAlignment() {
        return RequestedAlignment.AUTO; // TODO: calculate requested alignment
    }

    private synchronized Optional<TimedLEDState> handleLEDs(double timestamp) {
        Optional<TimedLEDState> state = handleFailureLEDs(timestamp);
        if (state.isEmpty()) {
            state = handleScoringAlignmentLEDs(timestamp);
        }
        if (state.isEmpty()) {
            state = handleIntakingLEDs(timestamp);
        }
        return state;
    }

    private synchronized Optional<TimedLEDState> handleIntakingLEDs(double timestamp) {
        boolean hasPiece = false; // TODO: determine if we have a gamepiece
        return Optional.of(hasPiece ? TimedLEDState.StaticLEDState.kHasGamepiece : TimedLEDState.RSLBasedLEDState.kWaitForGamepiece);
    }

    private synchronized Optional<TimedLEDState> handleScoringAlignmentLEDs(double timestamp) {
        LEDState pieceColor = LEDState.kGamepiece;
        // double maxError = 0.56 / 2.0; // m (distance between low goals)
        double maxError = 2;

        // Pose2d fieldToVehicle = RobotState.getInstance().getFieldToVehicleAbsolute(timestamp);
        Pose2d fieldToVehicle = RobotStateTracker.getInstance().getCurrentRobotPose();
        Optional<Pose2d> targetPoint = AutoAlignPointSelector.getAlignTarget(fieldToVehicle, getRequestedAlignment());

        if (targetPoint.isEmpty()) {
            // the target point will be empty if we are too far away from alignment, and we shouldn't hint alignment
            return Optional.empty();
        } else {
            Translation2d error = targetPoint.get().getTranslation().plus(fieldToVehicle.getTranslation().unaryMinus());
            double errorMagnitude = Math.abs(error.getNorm());
            boolean auto_align_active = RobotStateTracker.getInstance().getAutoAlignActive();
            boolean auto_align_on_target = RobotStateTracker.getInstance().getAutoAlignComplete();
            if ((errorMagnitude < Constants.kLEDClosenessDeadbandMeters && !auto_align_active) || (auto_align_active && auto_align_on_target)) {
                if (atGoal()) {
                    return Optional.of(TimedLEDState.StaticLEDState.kAtAlignment);
                }
                return Optional.of(TimedLEDState.BlinkingLEDState.kWaitingForDelivery);
            } else {
                if (errorMagnitude <= Constants.kLEDClosenessDeadbandMeters) {
                    errorMagnitude = 0.0;
                }
                double percentage = Util.limit((maxError - errorMagnitude) / maxError, 0.0, 1.0);
                return Optional.of(new TimedLEDState.PercentFullLEDState(percentage, pieceColor));
            }
        }
    }

    private synchronized Optional<TimedLEDState> handleFailureLEDs(double timestamp) {
        if (mForceFailure) {
            return Optional.of(TimedLEDState.StaticLEDState.kArmFailure);
        } else {
            return Optional.empty();
        }
    } 
}
