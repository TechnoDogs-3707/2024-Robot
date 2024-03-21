package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armTilt.ArmTilt;
import frc.robot.subsystems.armTilt.ArmTilt.ArmPositionPreset;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

public class ScoreAmpWithArm extends SequentialCommandGroup {
    public ScoreAmpWithArm(ObjectiveTracker objective, ArmTilt armTilt, IntakeDeploy intakeDeploy, Intake intake, BooleanSupplier overrideScore) {
        addCommands(
            // armTilt.setPositionCommand(ArmPositionPreset.AMP_SCORE),
            armTilt.setPositionBlockingCommand(ArmPositionPreset.AMP_SCORE),
            Commands.waitUntil(overrideScore),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.AMP_SCORE),
            intake.setActionCommand(IntakeWantedAction.REVERSE),
            Commands.waitSeconds(1),
            intake.setActionCommand(IntakeWantedAction.OFF),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.STOWED)
            .finallyDo(() -> {
                intake.setWantedAction(IntakeWantedAction.OFF);
                intakeDeploy.setPositionPreset(IntakePositionPreset.STOWED);
                armTilt.setPositionPreset(ArmPositionPreset.STOWED);
            })
        );
    }
}
