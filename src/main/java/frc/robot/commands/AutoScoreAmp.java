package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStateTracker;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.lib.drive.AutoAlignPointSelector.RequestedAlignment;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.AutoAlignScoreState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class AutoScoreAmp extends SequentialCommandGroup {
    public AutoScoreAmp(Drive drive, Arm arm, Intake intake, ObjectiveTracker objective, Supplier<Boolean> scoreOverride) {
        addCommands(
            Commands.waitUntil(drive::autoAlignAtTarget)
            .raceWith(Commands.waitUntil(scoreOverride::get))
            .alongWith(
                Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.SCORE_AMP_AUTOALIGN))
                .andThen(Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.DRIVING_TO_TARGET))),
                arm.setGoalCommand(GoalState.SCORE_AMP)
            ).andThen(
                Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.SCORE_RUNNING)),
                intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT),
                intake.waitUntilEmptyCommand(),
                Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.SCORE_FINISHED))
            ).finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                arm.setGoalState(GoalState.STOW);
                intake.setWantedAction(IntakeWantedAction.OFF);
            })
        );
    }
}
