package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class StopEverything extends SequentialCommandGroup {
    public StopEverything(IntakeDeploy intakeDeploy, Intake intake, Indexer indexer, Tilt tilt, Flywheels flywheels) {
        addCommands(
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.STOWED).alongWith(
                intake.setActionCommand(IntakeWantedAction.OFF),
                indexer.setActionCommand(IndexerWantedAction.OFF),
                flywheels.setActionCommand(FlywheelsWantedAction.OFF),
                tilt.setGoalCommand(TiltGoalState.STOW)
            )
        );
    }
}
