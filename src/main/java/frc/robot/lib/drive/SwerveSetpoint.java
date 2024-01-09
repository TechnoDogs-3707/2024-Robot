package frc.robot.lib.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public static final SwerveSetpoint FOUR_WHEEL_IDENTITY = buildIdentity(4);

    public ChassisSpeeds mChassisSpeeds;
    public SwerveModuleState[] mModuleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }

    public static SwerveSetpoint buildIdentity(int modules) {
        ChassisSpeeds speeds = new ChassisSpeeds();
        SwerveModuleState[] states = new SwerveModuleState[modules];
        for (int i = 0; i < modules; i++) {
            states[i] = new SwerveModuleState();
        }
        return new SwerveSetpoint(speeds, states);
    }

    @Override
    public String toString() {
        String ret = mChassisSpeeds.toString() + "\n";
        for (int i = 0; i < mModuleStates.length; ++i ) {
            ret += "  " + mModuleStates[i].toString() + "\n";
        }
        return ret;
    }
}
