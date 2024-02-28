package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class StopEverything extends SequentialCommandGroup {
    public StopEverything(Arm arm, Intake intake, Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels) {
        addCommands(
            arm.setGoalCommand(GoalState.STOW).alongWith(
                intake.setActionCommand(IntakeWantedAction.OFF),
                indexer.setActionCommand(IndexerWantedAction.OFF),
                flywheels.setActionCommand(FlywheelsWantedAction.OFF),
                tilt.setGoalCommand(ShooterTiltGoalState.STOW)
            )
        );
    }
}
