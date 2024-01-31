package frc.robot.subsystems.arm;

import frc.robot.lib.util.Util;

import static frc.robot.Constants.ArmSubsystem.*;

public class ArmState {
    public double j1 = J1.kHomePosition;
    public double j2 = J2.kHomePosition;
    public double j1Tolerance = J1.kLiberalAllowableError;
    public double j2Tolerance = J2.kLiberalAllowableError;
    public ArmSend send = ArmSend.MEDIUM;

    public enum ArmSend {
        LOW,
        MEDIUM,
        FULL
    }

    public ArmState(double j1, double j2, double j1Tolerance, double j2Tolerance, ArmSend send) {
        this.j1 = j1;
        this.j2 = j2;
        this.j1Tolerance = j1Tolerance;
        this.j2Tolerance = j2Tolerance;
        this.send = send;
    }

    public ArmState(ArmState other) {
        this(other.j1, other.j2, other.j1Tolerance, other.j2Tolerance, other.send);
    }

    public ArmState(double tilt, double wrist) {
        this(tilt, wrist, J1.kLiberalAllowableError, J2.kLiberalAllowableError, ArmSend.MEDIUM);
    }

    public ArmState(double tilt, double wrist, ArmSend send) {
        this(tilt, wrist, J1.kLiberalAllowableError, J2.kLiberalAllowableError, send);
    }

    public ArmState() {}

    public static ArmState withConservativeConstraints(double tilt, double wrist, ArmSend send) {
        return new ArmState(tilt, wrist, J1.kConservativeAllowableError, J2.kConservativeAllowableError, send);
    }

    public static ArmState withLiberalConstraints(double tilt, double wrist, ArmSend send) {
        return new ArmState(tilt, wrist, J1.kLiberalAllowableError, J2.kLiberalAllowableError, send);
    }

    public static ArmState generateWithFailsafeParameters(double tilt, double wrist) {
        return new ArmState(tilt, wrist, 
            J1.kConservativeAllowableError,
            J2.kConservativeAllowableError,
            ArmSend.LOW);
    }

    public double getJ1() {
        return Util.limit(j1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public double getJ2() {
        return Util.limit(j2, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public boolean isInRange(ArmState other) {
        return isInRange(other, Math.min(this.j1Tolerance, other.j1Tolerance), Math.min(this.j2Tolerance, other.j2Tolerance));
    }

    public boolean isInRange(ArmState other, double tiltAllowableError, double wristAllowableError) {
        return Util.epsilonEquals(this.getJ1(), other.getJ1(), tiltAllowableError)
                && Util.epsilonEquals(this.getJ2(), other.getJ2(), wristAllowableError);
    }

    public String toString() {
        return "Arm state: (" + getJ1() + ", " + getJ2() + ")";
    }
}
