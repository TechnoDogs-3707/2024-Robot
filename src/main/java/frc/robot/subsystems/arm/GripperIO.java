package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.arm.Arm.GameObjectType;

public interface GripperIO {
    @AutoLog
    public static class GripperIOInputs {
        public double motorSpeedRotationsPerSecond = 0.0;
        public double suppliedCurrentAmps = 0.0;
        public double hottestMotorTempCelsius = 0.0;
        public boolean cubeInIntake = false;
        public boolean coneInIntake = false;
    }

    public default void updateInputs(GripperIOInputs inputs) {}

    public default void updateOutputs() {}
    
    /**
     * Set the throttle of the intake motor.
     * @param throttle
     */
    public default void setMotor(double throttle) {}

    /**
     * Set the gripper to cone mode. 
     * When intaking cones, the gripper may need to use an alternate motor, 
     * or run the main motor in reverse. If it is required, the {@link GripperIO}
     * class provides this method to inform the implementation which intake mode
     * is selected.
     * @param coneMode
     */
    public default void setGameObject(GameObjectType object) {}
}
