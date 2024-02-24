package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntakeStateMachine.ArmIntakeWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class ShooterTesting {
    private ShooterTesting() {
        throw new IllegalStateException("This class can not be instantiated.");
    }

    public static class ShooterTurnOn extends Command {
        private ShooterFlywheels mFlywheels;
        private ShooterTilt mTilt;

        public ShooterTurnOn(ShooterFlywheels flywheels, ShooterTilt tilt) {
            mFlywheels = flywheels;
            mTilt = tilt;
            addRequirements(flywheels, tilt);
        }

        @Override
        public void initialize() {
            mTilt.setGoalState(ShooterTiltGoalState.PODIUM);
            mFlywheels.setSetpointSpeedLeft(20);
            mFlywheels.setSetpointSpeedRight(40);
            mFlywheels.setWantedAction(ShooterFlywheelsStateMachine.FlywheelsWantedAction.SHOOT);
        }

        @Override
        public void end(boolean interrupted) {
            mTilt.setGoalState(ShooterTiltGoalState.STOW);
            mFlywheels.setWantedAction(ShooterFlywheelsStateMachine.FlywheelsWantedAction.OFF);
        }
    }

    public static class IndexerLoadGamepiece extends Command {
        private Indexer mIndexer;

        public IndexerLoadGamepiece(Indexer indexer) {
            mIndexer = indexer;
            addRequirements(indexer);
        }

        @Override
        public void initialize() {
            mIndexer.setWantedAction(IndexerStateMachine.IndexerWantedAction.INTAKE);
        }

        @Override
        public void end(boolean interrupted) {
            mIndexer.setWantedAction(IndexerStateMachine.IndexerWantedAction.OFF);
        }
    }

    public static class IndexerScoreGampiece extends Command {
        private Indexer mIndexer;

        public IndexerScoreGampiece(Indexer indexer) {
            mIndexer = indexer;
            addRequirements(indexer);
        }

        @Override
        public void initialize() {
            mIndexer.setWantedAction(IndexerStateMachine.IndexerWantedAction.SCORE);
        }

        @Override
        public void end(boolean interrupted) {
            mIndexer.setWantedAction(IndexerStateMachine.IndexerWantedAction.OFF);
        }
    }

    public static class JamClear extends Command {
        private Indexer mIndexer;
        private Arm mArm;
        // private ShooterFlywheels mFlywheels;
        // private ShooterTilt mTilt;

        public JamClear(Indexer indexer, Arm arm/*, ShooterFlywheels flywheels, ShooterTilt tilt*/) {
            mArm = arm;
            mIndexer = indexer;
            // mFlywheels = flywheels;
            // mTilt = tilt;
            addRequirements(indexer, arm/*, flywheels, tilt*/);
        }

        @Override
        public void initialize() {
            // mFlywheels.setWantedAction(ShooterFlywheelsStateMachine.FlywheelsWantedAction.OFF);
            // mTilt.setGoalState(ShooterTiltGoalState.STOW);
            mIndexer.setWantedAction(IndexerStateMachine.IndexerWantedAction.REVERSE);
            mArm.setWantedAction(ArmIntakeWantedAction.REVERSE);

        }

        @Override
        public void end(boolean interrupted) {
            // mFlywheels.setWantedAction(ShooterFlywheelsStateMachine.FlywheelsWantedAction.OFF);
            // mTilt.setGoalState(ShooterTiltGoalState.STOW);
            mIndexer.setWantedAction(IndexerStateMachine.IndexerWantedAction.OFF);
            mArm.setWantedAction(ArmIntakeWantedAction.OFF);
        }
    }

    public static class StowEverything extends Command {
        private Indexer mIndexer;
        private ShooterFlywheels mFlywheels;
        private ShooterTilt mTilt;

        public StowEverything(Indexer indexer, ShooterFlywheels flywheels, ShooterTilt tilt) {
            mIndexer = indexer;
            mFlywheels = flywheels;
            mTilt = tilt;
            addRequirements(indexer, flywheels, tilt);
        }

        @Override
        public void initialize() {
            mFlywheels.setWantedAction(ShooterFlywheelsStateMachine.FlywheelsWantedAction.OFF);
            mTilt.setGoalState(ShooterTiltGoalState.STOW);
            mIndexer.setWantedAction(IndexerStateMachine.IndexerWantedAction.OFF);
        }
    }
}
