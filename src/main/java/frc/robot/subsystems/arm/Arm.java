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
import frc.robot.lib.util.TimeDelayedBoolean;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.arm.ArmState.ArmAction;
import frc.robot.subsystems.arm.ArmState.ArmSend;
import frc.robot.subsystems.leds.LED;

public class Arm extends SubsystemBase {

    public enum GameObjectType {
        CUBE, CONE
    }

    public enum GoalState {
        STOW(ArmState.withConservativeConstraints(0, 0, 0, ArmAction.NEUTRAL, ArmSend.LOW)),
        TRANSPORT(ArmState.withConservativeConstraints(0.03, 0, 0.03, ArmAction.NEUTRAL, ArmSend.MEDIUM)),
        INTAKE_CUBE_GROUND(ArmState.withConservativeConstraints(0, 0, 0, ArmAction.INTAKING, ArmSend.LOW)),
        INTAKE_CUBE_SHELF(ArmState.withConservativeConstraints(0.14, 0.82, 0.2, ArmAction.INTAKING, ArmSend.MEDIUM)),
        INTAKE_CONE_GROUND(ArmState.withConservativeConstraints(0.03, 0.0, 0.38, ArmAction.INTAKING, ArmSend.MEDIUM)),
        INTAKE_CONE_SHELF(ArmState.withConservativeConstraints(0.143, 0.78, 0.48, ArmAction.INTAKING, ArmSend.MEDIUM)),
        INTAKE_WAIT_SHELF(ArmState.withConservativeConstraints(0.143, 0, 0.2, ArmAction.NEUTRAL, ArmSend.FULL)),
        SCORE_WAIT(ArmState.withLiberalConstraints(0.12, 0, 0.12, ArmAction.NEUTRAL, ArmSend.FULL)),
        SCORE_WAIT_LOW(ArmState.withLiberalConstraints(0.08, 0, 0.12, ArmAction.NEUTRAL, ArmSend.FULL)),
        SCORE_CUBE_LOW(ArmState.withConservativeConstraints(0.05, 0, 0, ArmAction.SCORING, ArmSend.MEDIUM)),
        SCORE_CUBE_MID(ArmState.withConservativeConstraints(0.12, 0.75, 0.16, ArmAction.SCORING, ArmSend.MEDIUM)),
        SCORE_CUBE_HIGH(ArmState.withConservativeConstraints(0.13, 1.1, 0.13, ArmAction.SCORING, ArmSend.MEDIUM)),
        SCORE_CONE_LOW(ArmState.withConservativeConstraints(0.05, 0, 0.3, ArmAction.SCORING, ArmSend.MEDIUM)),
        SCORE_CONE_MID(ArmState.withConservativeConstraints(0.14, 0.7, 0.48, ArmAction.SCORING, ArmSend.MEDIUM)),
        SCORE_CONE_HIGH(ArmState.withConservativeConstraints(0.13, 1.1, 0.42, ArmAction.SCORING, ArmSend.MEDIUM));
        
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
    private GameObjectType mGameObject = GameObjectType.CUBE;
    private TimeDelayedBoolean mEnsureScoringFinished = new TimeDelayedBoolean();
    private TimeDelayedBoolean mEnsureIntakeFinished = new TimeDelayedBoolean();
    private boolean mResetMotionPlanner = false;
    private boolean mGripperHasGamepiece = false;

    private boolean mForceFailure = false;

    public static final double kArmBaseLength = Units.inchesToMeters(19); // distance from arm pivot to vertical
                                                                           // extension of wrist pivot
    public static final double kArmMassOffset = 4.0; // FF Amps required to hold elevator horizontal at minimum
                                                      // extension and wrist vertical
    public static final double kArmMassDistanceFactor = 8.0; // Additional FF Amps to hold horizontal required per
                                                               // meter of extension
    public static final double kArmConeBoost = 2.0; // FF Amps to add to tilt when wrist is at full extension
    public static final double kElevatorMassFactor = 18.0; // FF Amps to hold elevator carriage in vertical position
    public static final double kWristMassFactor = 6.5; // FF Amps to hold wrist in horizontal position

    private final Mechanism2d mSensorMech = new Mechanism2d(1.75, 1.75);
    private final MechanismRoot2d mSensorMechRoot = mSensorMech.getRoot("Main Pivot", 0.25, 0.25);
    private final MechanismLigament2d mSensorMechElevator = mSensorMechRoot
            .append(new MechanismLigament2d("Elevator", kArmBaseLength, 0, 5, new Color8Bit(Color.kOrange)));
    private final MechanismLigament2d mSensorMechOffsetPlate = mSensorMechElevator
            .append(new MechanismLigament2d("Offset Plate", 0.08, 270, 5, new Color8Bit(Color.kGray)));
    private final MechanismLigament2d mSensorMechWrist = mSensorMechOffsetPlate
            .append(new MechanismLigament2d("Wrist", Units.inchesToMeters(12), 100, 5, new Color8Bit(Color.kPurple)));

    private final Mechanism2d mTargetMech = new Mechanism2d(1.75, 1.75);
    private final MechanismRoot2d mTargetMechRoot = mTargetMech.getRoot("Main Pivot", 0.25, 0.25);
    private final MechanismLigament2d mTargetMechElevator = mTargetMechRoot
            .append(new MechanismLigament2d("Elevator", kArmBaseLength, 0, 5, new Color8Bit(Color.kLightGray)));
    private final MechanismLigament2d mTargetMechOffsetPlate = mTargetMechElevator
            .append(new MechanismLigament2d("Offset Plate", 0.08, 270, 5, new Color8Bit(Color.kLightGray)));
    private final MechanismLigament2d mTargetMechWrist = mTargetMechOffsetPlate
            .append(new MechanismLigament2d("Wrist", Units.inchesToMeters(12), 100, 5, new Color8Bit(Color.kLightGray)));

    private ArmIO mArmIO;
    private ArmIOInputsAutoLogged mArmInputs;

    private GripperIO mGripperIO;
    private GripperIOInputsAutoLogged mGripperInputs;

    public Arm(ArmIO armIO, GripperIO gripperIO) {
        mMotionPlanner = new ArmMotionPlanner();

        mArmIO = armIO;
        mArmInputs = new ArmIOInputsAutoLogged();

        mGripperIO = gripperIO;
        mGripperInputs = new GripperIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        mArmIO.updateInputs(mArmInputs);
        mGripperIO.updateInputs(mGripperInputs);
        Logger.processInputs("Arm", mArmInputs);
        Logger.processInputs("Gripper", mGripperInputs);

        double timestamp = Timer.getFPGATimestamp();

        Optional<TimedLEDState> ledState = handleLEDs(timestamp);
        if (DriverStation.isEnabled()) {
            if (ledState.isPresent()) {
                if (DriverStation.isAutonomous()) {
                    LED.setWantedAction(LED.WantedAction.DISPLAY_VISION);
                } else {
                    LED.setDeliveryLEDState(ledState.get());
                    LED.setWantedAction(LED.WantedAction.DISPLAY_DELIVERY);
                }
            } else {
                LED.setWantedAction(LED.WantedAction.OFF);
            }
        }
        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            simCurrent += mArmInputs.tiltSuppliedCurrentAmps;
            simCurrent += mArmInputs.extendSuppliedCurrentAmps;
            simCurrent += mArmInputs.wristSuppliedCurrentAmps;
            simCurrent += mGripperInputs.suppliedCurrentAmps;

            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        mGripperHasGamepiece = mGripperInputs.coneInIntake || mGripperInputs.cubeInIntake;
        Logger.recordOutput("Arm/Gripper/HasGamepiece", mGripperHasGamepiece);
        Logger.recordOutput("Arm/Gripper/FinishedIntaking", isDoneIntaking());
        Logger.recordOutput("Arm/Gripper/FinishedScoring", isDoneScoring());

        Rotation2d tiltAngle = Rotation2d.fromRotations(mArmInputs.tiltRotations);
        Rotation2d wristAngle = Rotation2d.fromRotations(mArmInputs.wristRotations);

        mSensorMechElevator.setAngle(tiltAngle);
        mSensorMechElevator.setLength(kArmBaseLength + mArmInputs.extendMeters);
        mSensorMechWrist.setAngle(wristAngle.unaryMinus().minus(Rotation2d.fromDegrees(100)));

        mMeasuredState = new ArmState(mArmInputs.tiltRotations, mArmInputs.extendMeters, mArmInputs.wristRotations, mCommandedState.action, mCommandedState.send);

        ArmState nextArmState = mMotionPlanner.update(mMeasuredState);
        mLastCommandedState = mCommandedState;
        mCommandedState = nextArmState;

        Logger.recordOutput("Arm/GoalState/Name", mGoalState.name());
        Logger.recordOutput("Arm/CommandedState/Action", mCommandedState.action.name());
        Logger.recordOutput("Arm/RequestedAlignment", getRequestedAlignment().name());
        Logger.recordOutput("Arm/GoalState/Action", mGoalState.state.action.name());
        Logger.recordOutput("Arm/GoalState/Send", mGoalState.state.send.name());
        Logger.recordOutput("Arm/CommandedState/Tolerance/Tilt", mCommandedState.tiltTolerance);
        Logger.recordOutput("Arm/CommandedState/Tolerance/Extend", mCommandedState.extendTolerance);
        Logger.recordOutput("Arm/CommandedState/Tolerance/Wrist", mCommandedState.wristTolerance);

        // set correct game object type
        mGripperIO.setGameObject(mGameObject);
        Logger.recordOutput("Arm/Gripper/GameObject", mGameObject.name());

        // interpret and execute action from next arm state
        double gripperCommandedOutput = 0;

        switch (mCommandedState.action) {
            case INTAKING:
                if (!isDoneIntaking()) {
                    gripperCommandedOutput = -1;
                } else {
                    gripperCommandedOutput = 0;
                }
                break;
            case SCORING:
                if (atGoal() && !isDoneScoring()) {
                    gripperCommandedOutput = 1;
                } else {
                    gripperCommandedOutput = 0;
                }
                break;
            case NEUTRAL:
            default:
                gripperCommandedOutput = 0;
                break;
        }

        mGripperIO.setMotor(gripperCommandedOutput);
        Logger.recordOutput("Arm/Gripper/CommandedOutput", gripperCommandedOutput);

        mTargetMechElevator.setAngle(Rotation2d.fromRotations(mCommandedState.tilt));
        mTargetMechElevator.setLength(kArmBaseLength + mCommandedState.extend);
        mTargetMechWrist.setAngle(Rotation2d.fromRotations(mCommandedState.wrist).unaryMinus().minus(Rotation2d.fromDegrees(100)));

        Logger.recordOutput("Arm/MeasuredPositions", mSensorMech);
        Logger.recordOutput("Arm/TargetPositions", mTargetMech);
        Logger.recordOutput("Arm/MotionPlanner/AtGoal", atGoal());
        Logger.recordOutput("Arm/MotionPlanner/StatesRemaining", mMotionPlanner.getRemainingStates());

        var currentPose = RobotStateTracker.getInstance().getCurrentRobotPose();
        RobotStateTracker.getInstance().setAutoAlignReady(AutoAlignPointSelector.chooseTargetPoint(currentPose, getRequestedAlignment()).isPresent());

        if (mResetMotionPlanner) {
            mMotionPlanner.reset();
        }

        if (mCommandedState.tilt != mLastCommandedState.tilt) {
            mArmIO.setTiltTarget(mCommandedState.tilt);
        }
        double tiltFeedForward = calcTiltFeedforward();
        Logger.recordOutput("Arm/CalculatedFeedForwards/Tilt", tiltFeedForward);
        mArmIO.setTiltFeedForward(tiltFeedForward);

        if (mCommandedState.extend != mLastCommandedState.extend) {
            mArmIO.setExtendTarget(mCommandedState.extend);
        }
        double extendFeedForward = calcExtendFeedForward();
        Logger.recordOutput("Arm/CalculatedFeedForwards/Extend", extendFeedForward);
        mArmIO.setExtendFeedForward(extendFeedForward);

        if (mCommandedState.wrist != mLastCommandedState.wrist ) {
            mArmIO.setWristTarget(mCommandedState.wrist);
        }
        double wristFeedForward = calcWristFeedForward();
        Logger.recordOutput("Arm/CalculatedFeedForwards/Wrist", wristFeedForward);
        mArmIO.setWristFeedForward(wristFeedForward);

        mArmIO.updateOutputs();
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

    private Rotation2d calcWristRefAngle() {
        return Rotation2d.fromRotations(mArmInputs.wristRotations).minus(Rotation2d.fromRotations(mArmInputs.tiltRotations));
    }

    private double calcTiltFeedforward() {
        Rotation2d tiltAngle = Rotation2d.fromRotations(mArmInputs.tiltRotations);
        Rotation2d wristAngle = Rotation2d.fromRotations(mArmInputs.wristRotations);
        double extendDistance = mArmInputs.extendMeters;

        // estimates a value proportional to the torque on each tilt gearbox
        double wristMass = wristAngle.getCos() * kArmConeBoost;
        double tiltMass = kArmMassOffset + (extendDistance * kArmMassDistanceFactor);
        double tiltFFCurrent = (wristMass + tiltMass) * tiltAngle.getCos();

        return tiltFFCurrent;
    }

    private double calcExtendFeedForward() {
        Rotation2d tiltAngle = Rotation2d.fromRotations(mArmInputs.tiltRotations);
        
        // estimates a value proportional to the torque on each extend gearbox
        double extendFeedForward = kElevatorMassFactor * tiltAngle.getSin();

        return extendFeedForward;
    }

    private double calcWristFeedForward() {
        return calcWristRefAngle().getCos() * kWristMassFactor;
    }

    public GameObjectType getGameObject() {
        return mGameObject;
    }

    public boolean gripperHasGamepiece() {
        return mGripperHasGamepiece;
    }

    public boolean isDoneIntaking() {
        return mEnsureIntakeFinished.update(gripperHasGamepiece(), 0.5);
    }

    public boolean isDoneScoring() {
        return mEnsureScoringFinished.update(!gripperHasGamepiece(), 0.5);
    }

    public boolean gameObjectIsCone() {
        return this.mGameObject == GameObjectType.CONE;
    }

    public void setGameObject(GameObjectType gameObject) {
        this.mGameObject = gameObject;
    }

    public void swapGameObject() {
        if (gameObjectIsCone()) {
            mGameObject = GameObjectType.CUBE;
        } else {
            mGameObject = GameObjectType.CONE;
        }
    }

    public RequestedAlignment getRequestedAlignment() {
        boolean isCone = gameObjectIsCone();
        boolean isScoreLow = getGoalState() == GoalState.SCORE_CONE_LOW || getGoalState() == GoalState.SCORE_CUBE_LOW || getGoalState() == GoalState.SCORE_WAIT_LOW;

        if (isScoreLow) {
            return RequestedAlignment.AUTO;
        } else {
            if (isCone) {
                return RequestedAlignment.AUTO_CONE;
            } else {
                return RequestedAlignment.AUTO_CUBE;
            }
        }
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
        Optional<TimedLEDState> display = Optional.empty();
        boolean hasPiece = false;
        switch (mGameObject) {
            case CONE:
                hasPiece = gripperHasGamepiece();
                display = Optional.of(hasPiece ? TimedLEDState.StaticLEDState.kHasCone : TimedLEDState.RSLBasedLEDState.kConeIntakeWaiting);
                break;
            case CUBE:
                hasPiece = gripperHasGamepiece();
                display = Optional.of(hasPiece ? TimedLEDState.StaticLEDState.kHasCube : TimedLEDState.RSLBasedLEDState.kCubeIntakeWaiting);
                break;
        }
        return display;
    }

    private synchronized Optional<TimedLEDState> handleScoringAlignmentLEDs(double timestamp) {
        AutoAlignPointSelector.RequestedAlignment alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO;
        LEDState pieceColor = LEDState.kAutoAlign;
        double maxError = 0.56 / 2.0; // m (distance between low goals)
        switch (mGameObject) {
            case CONE:
                alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO_CONE;
                pieceColor = LEDState.kIntakingCone;
                maxError = 1.13 / 2.0; // m (distance between max cones)
                break;
            case CUBE:
                alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO_CUBE;
                pieceColor = LEDState.kIntakingCube;
                maxError = 1.68 / 2.0; // m (distance between tags)
                break;
        }

        if (mGoalState == GoalState.SCORE_CONE_LOW || mGoalState == GoalState.SCORE_CUBE_LOW || mGoalState == GoalState.SCORE_WAIT_LOW) {
            // if we aren't low scoring, we need to pick cones or cubes for alignment hinter
            alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO;
        }
        // Pose2d fieldToVehicle = RobotState.getInstance().getFieldToVehicleAbsolute(timestamp);
        Pose2d fieldToVehicle = RobotStateTracker.getInstance().getCurrentRobotPose();
        Optional<Pose2d> targetPoint = AutoAlignPointSelector.chooseTargetPoint(fieldToVehicle, alignmentType);

        if (targetPoint.isEmpty()) {
            // the target point will be empty if we are too far away from alignment, and we shouldn't hint alignment
            return Optional.empty();
        } else {
            Translation2d error = targetPoint.get().getTranslation().plus(fieldToVehicle.getTranslation().unaryMinus());
            double horizError = Math.abs(error.getY());
            boolean auto_align_active = RobotStateTracker.getInstance().getAutoAlignActive();
            boolean auto_align_on_target = RobotStateTracker.getInstance().getAutoAlignComplete();
            if ((horizError < Constants.kLEDClosenessDeadbandMeters && !auto_align_active) || (auto_align_active && auto_align_on_target)) {
                return Optional.of(TimedLEDState.StaticLEDState.kAtAlignment);
            } else {
                if (horizError <= Constants.kLEDClosenessDeadbandMeters) {
                    horizError = 0.0;
                }
                double percentage = Util.limit((maxError - horizError) / maxError, 0.0, 1.0);
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
