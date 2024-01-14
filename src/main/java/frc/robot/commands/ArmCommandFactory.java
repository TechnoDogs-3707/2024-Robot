package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class ArmCommandFactory {
    public static final Command setAlignStateOverride(boolean value, Drive drive) {
        return new InstantCommand(() -> drive.setAlignStateOverride(value));
    }

    public static final Command alignStateOverrideButton(Drive drive) {
        return new StartEndCommand(() -> drive.setAlignStateOverride(true), () -> drive.setAlignStateOverride(false));
    }

    public static final Command armFailureSwitch(Arm arm) {
        return new StartEndCommand(() -> arm.setForceFailure(true), () -> arm.setForceFailure(false));
    }
}
