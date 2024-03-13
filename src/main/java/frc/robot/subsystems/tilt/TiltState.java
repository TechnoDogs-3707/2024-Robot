package frc.robot.subsystems.tilt;

import static frc.robot.Constants.ShooterTilt.*;

import frc.robot.util.poofsUtils.PoofsUtil;

public class TiltState {
    public double defaultPosition = kHomePosition;
    public boolean autoAim = false;
    public boolean strictPositionTolerance = false;

    public TiltState(double defaultPosition, boolean autoAim, boolean strictPositionTolerance) {
        this.defaultPosition = defaultPosition;
        this.autoAim = autoAim;
        this.strictPositionTolerance = strictPositionTolerance;
    }

    public TiltState() {}

    public boolean isInRange(TiltState other) {
        return 
            PoofsUtil.epsilonEquals(
                defaultPosition, 
                other.defaultPosition, 
                this.strictPositionTolerance || other.strictPositionTolerance ? kConservativeAllowableError : kLiberalAllowableError
            ) &&
            this.autoAim == other.autoAim &&
            this.strictPositionTolerance == other.strictPositionTolerance;
    }
}
