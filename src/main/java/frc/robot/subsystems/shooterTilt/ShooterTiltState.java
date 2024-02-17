package frc.robot.subsystems.shooterTilt;

import static frc.robot.Constants.ShooterTilt.*;

import frc.robot.lib.util.Util;

public class ShooterTiltState {
    public double defaultPosition = kHomePosition;
    public boolean autoAim = false;
    public boolean strictPositionTolerance = false;

    public ShooterTiltState(double defaultPosition, boolean autoAim, boolean strictPositionTolerance) {
        this.defaultPosition = defaultPosition;
        this.autoAim = autoAim;
        this.strictPositionTolerance = strictPositionTolerance;
    }

    public ShooterTiltState() {}

    public boolean isInRange(ShooterTiltState other) {
        return 
            Util.epsilonEquals(
                defaultPosition, 
                other.defaultPosition, 
                this.strictPositionTolerance || other.strictPositionTolerance ? kConservativeAllowableError : kLiberalAllowableError
            ) &&
            this.autoAim == other.autoAim &&
            this.strictPositionTolerance == other.strictPositionTolerance;
    }
}
