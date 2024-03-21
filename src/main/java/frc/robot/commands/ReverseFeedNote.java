package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armTilt.ArmTilt;
import frc.robot.subsystems.armTilt.ArmTilt.ArmPositionPreset;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class ReverseFeedNote extends SequentialCommandGroup {
    public ReverseFeedNote(ArmTilt armTilt, Indexer indexer, ObjectiveTracker objectiveTracker, IntakeDeploy intakeDeploy, Intake intake, Flywheels flywheels, Tilt tilt) {
        addCommands(
            flywheels.setActionCommand(FlywheelsWantedAction.OFF),
            tilt.setGoalCommand(TiltGoalState.STOW),
            indexer.setActionCommand(IndexerWantedAction.OFF),
            intake.setActionCommand(IntakeWantedAction.OFF),
            armTilt.setPositionBlockingCommand(ArmPositionPreset.STOWED),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.DEPLOYED),
            intake.setActionCommand(IntakeWantedAction.REVERSE)
            .alongWith(indexer.setActionCommand(IndexerWantedAction.REVERSE)),
            Commands.waitSeconds(1),
            intake.setActionCommand(IntakeWantedAction.OFF),
            indexer.setActionCommand(IndexerWantedAction.OFF),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.STOWED)
        );
    }
}
