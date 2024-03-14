package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;

public class IndexerJamClearing extends SequentialCommandGroup {
    public IndexerJamClearing(IntakeDeploy intakeDeploy, Intake intake, Indexer indexer) { //TODO: jam clear LED state
        addCommands(
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.EJECT).andThen(
                indexer.setActionCommand(IndexerWantedAction.REVERSE),
                intake.setActionCommand(IntakeWantedAction.REVERSE),
                Commands.idle()
            ).finallyDo(() -> {
                indexer.setWantedAction(IndexerWantedAction.OFF);
                intake.setWantedAction(IntakeWantedAction.OFF);
                intakeDeploy.setPositionPreset(IntakePositionPreset.STOWED);
            })
        );
    }
}
