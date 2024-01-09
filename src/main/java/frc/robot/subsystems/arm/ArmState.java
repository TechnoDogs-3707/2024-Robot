package frc.robot.subsystems.arm;

import frc.robot.lib.util.Util;

import static frc.robot.Constants.ArmSubsystem.*;

public class ArmState {
    public double tilt = Tilt.kHomePosition;
    public double extend = Extend.kHomePosition;
    public double wrist = Wrist.kHomePosition;
    public double tiltTolerance = Tilt.kLiberalAllowableError;
    public double extendTolerance = Extend.kLiberalAllowableError;
    public double wristTolerance = Wrist.kLiberalAllowableError;
    public ArmAction action = ArmAction.NEUTRAL;
    public ArmSend send = ArmSend.MEDIUM;

    /**  */
    public enum ArmAction {
        /** stop intake rollers */
        NEUTRAL,
        /** Run gripper out */
        SCORING,
        /** Run gripper in */
        INTAKING
    }

    public enum ArmSend {
        LOW,
        MEDIUM,
        FULL
    }

    public ArmState(double tilt, double extend, double wrist, double tiltTolerance, double extendTolerance, double wristTolerance, ArmAction action, ArmSend send) {
        this.tilt = tilt;
        this.extend = extend;
        this.wrist = wrist;
        this.tiltTolerance = tiltTolerance;
        this.extendTolerance = extendTolerance;
        this.wristTolerance = wristTolerance;
        this.action = action;
        this.send = send;
    }

    public ArmState(ArmState other) {
        this(other.tilt, other.extend, other.wrist, other.tiltTolerance, other.extendTolerance, other.wristTolerance, other.action, other.send);
    }

    public ArmState(double tilt, double extend, double wrist) {
        this(tilt, extend, wrist, Tilt.kLiberalAllowableError, Extend.kLiberalAllowableError, Wrist.kLiberalAllowableError, ArmAction.NEUTRAL, ArmSend.MEDIUM);
    }

    public ArmState(double tilt, double extend, double wrist, ArmAction action) {
        this(tilt, extend, wrist, Tilt.kLiberalAllowableError, Extend.kLiberalAllowableError, Wrist.kLiberalAllowableError, action, ArmSend.MEDIUM);
    }

    public ArmState(double tilt, double extend, double wrist, ArmAction action, ArmSend send) {
        this(tilt, extend, wrist, Tilt.kLiberalAllowableError, Extend.kLiberalAllowableError, Wrist.kLiberalAllowableError, action, send);
    }

    public ArmState() {}

    public static ArmState withConservativeConstraints(double tilt, double extend, double wrist, ArmAction action, ArmSend send) {
        return new ArmState(tilt, extend, wrist, Tilt.kConservativeAllowableError, Extend.kConservativeAllowableError, Wrist.kConservativeAllowableError, action, send);
    }

    public static ArmState withLiberalConstraints(double tilt, double extend, double wrist, ArmAction action, ArmSend send) {
        return new ArmState(tilt, extend, wrist, Tilt.kLiberalAllowableError, Extend.kLiberalAllowableError, Wrist.kLiberalAllowableError, action, send);
    }

    public static ArmState generateWithFailsafeParameters(double tilt, double extend, double wrist) {
        return new ArmState(tilt, extend, wrist, 
            Tilt.kConservativeAllowableError, 
            Extend.kConservativeAllowableError, 
            Wrist.kConservativeAllowableError, 
            ArmAction.NEUTRAL, ArmSend.LOW);
    }

    public double getTilt() {
        return Util.limit(tilt, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public double getExtend() {
        return Util.limit(extend, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public double getWrist() {
        return Util.limit(wrist, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public boolean isInRange(ArmState other) {
        return isInRange(other, Math.min(this.tiltTolerance, other.tiltTolerance), Math.min(this.extendTolerance, other.extendTolerance), Math.min(this.wristTolerance, other.wristTolerance));
    }

    public boolean isInRange(ArmState other, double tiltAllowableError, double extendAllowableError, double wristAllowableError) {
        return Util.epsilonEquals(this.getTilt(), other.getTilt(), tiltAllowableError)
                && Util.epsilonEquals(this.getExtend(), other.getExtend(), extendAllowableError)
                && Util.epsilonEquals(this.getWrist(), other.getWrist(), wristAllowableError);
    }

    public String toString() {
        return "Arm state: (" + getTilt() + ", " + getExtend() + ", " + getWrist() + ", " + action.toString() + ")";
    }
}
