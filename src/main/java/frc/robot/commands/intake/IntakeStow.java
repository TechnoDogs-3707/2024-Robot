package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armTilt.ArmTilt;
import frc.robot.subsystems.armTilt.ArmTilt.ArmPositionPreset;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;

public class IntakeStow extends SequentialCommandGroup {
    public IntakeStow(IntakeDeploy intakeDeploy, Intake intake, ArmTilt armTilt) {
        addCommands(
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.STOWED),
            armTilt.setPositionBlockingCommand(ArmPositionPreset.STOWED)
            .alongWith(
                intake.setActionCommand(IntakeWantedAction.OFF)
            )
        );
    }
}
