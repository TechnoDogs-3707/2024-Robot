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
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.ArmScoreAmpState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class ScoreAmpWithArm extends SequentialCommandGroup {
    public ScoreAmpWithArm(ObjectiveTracker objective, ArmTilt armTilt, IntakeDeploy intakeDeploy, Intake intake, BooleanSupplier overrideScore) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.ARM_SCORE_AMP)),
            Commands.runOnce(() -> objective.setArmScoreAmpState(ArmScoreAmpState.PREPARING)),
            armTilt.setPositionBlockingCommand(ArmPositionPreset.AMP_SCORE),
            Commands.runOnce(() -> objective.setArmScoreAmpState(ArmScoreAmpState.WAITING)),
            intakeDeploy.setPositionCommand(IntakePositionPreset.AMP_SCORE),
            Commands.waitUntil(overrideScore),
            Commands.runOnce(() -> objective.setArmScoreAmpState(ArmScoreAmpState.SCORING)),
            intake.setActionCommand(IntakeWantedAction.REVERSE),
            Commands.waitSeconds(0.125),
            intakeDeploy.setPositionCommand(IntakePositionPreset.AMP_SCORE_PRIME),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> objective.setArmScoreAmpState(ArmScoreAmpState.DONE)),
            intake.setActionCommand(IntakeWantedAction.OFF),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.STOWED),
            armTilt.setPositionBlockingCommand(ArmPositionPreset.STOWED),
            Commands.runOnce(() -> objective.setArmScoreAmpState(ArmScoreAmpState.DONE))
            .finallyDo(() -> {
                intake.setWantedAction(IntakeWantedAction.OFF);
                intakeDeploy.setPositionPreset(IntakePositionPreset.STOWED);
                armTilt.setPositionPreset(ArmPositionPreset.STOWED);
            })
        );
    }
}
