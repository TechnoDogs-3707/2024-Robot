package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntakeStateMachine;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class ScoringCommands {
    public static final BooleanSupplier armIsAtGoalState(Arm arm, GoalState goal) {
        return () -> arm.getMeasuredState().isInRange(goal.state);
    }

    public static final BooleanSupplier intakeIsSystemState(Arm arm, ArmIntakeStateMachine.SystemState state) {
        return () -> arm.getIntakeSystemState().equals(state);
    }

    public static final BooleanSupplier indexerIsSystemState(Indexer indexer, IndexerStateMachine.SystemState state) {
        return () -> indexer.getSystemState().equals(state);
    }

    public static final BooleanSupplier shooterIsAtGoalState(ShooterTilt tilt, ShooterTiltGoalState goal) {
        return () -> tilt.getGoalState().equals(goal);
    }

    public static final BooleanSupplier flywheelIsSystemState(ShooterFlywheels flywheels, ShooterFlywheelsStateMachine.SystemState state) {
        return () -> flywheels.getSystemState().equals(state);
    }

    public static Command sensorIntakeGroundToIndexer(Arm arm, Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> flywheels.setWantedAction(ShooterFlywheelsStateMachine.WantedAction.OFF), flywheels),
            new InstantCommand(() -> tilt.setGoalState(ShooterTiltGoalState.STOW), tilt),
            new InstantCommand(() -> indexer.setWantedAction(IndexerStateMachine.WantedAction.INTAKE), indexer),
            new InstantCommand(() -> arm.setGoalState(Arm.GoalState.INTAKE_GROUND), arm),
            new InstantCommand(() -> arm.setWantedAction(ArmIntakeStateMachine.WantedAction.INTAKE_HANDOFF), arm),
            new WaitUntilCommand(indexer::temporaryHasGamepiece),
            new InstantCommand(() -> arm.setWantedAction(ArmIntakeStateMachine.WantedAction.OFF), arm),
            new InstantCommand(() -> arm.setGoalState(Arm.GoalState.STOW), arm),
            new InstantCommand(() -> flywheels.setWantedAction(ShooterFlywheelsStateMachine.WantedAction.IDLE), flywheels)
        );
    }
}
