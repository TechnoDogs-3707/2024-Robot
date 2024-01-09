package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GameObjectType;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.drive.Drive;

public class ArmCommandFactory {
    public static final Command toggleGamepiece(Arm arm) {
        return new InstantCommand(arm::swapGameObject, arm);
    }

    public static final Command setGamepieceCone(Arm arm) {
        return new InstantCommand(() -> arm.setGameObject(GameObjectType.CONE), arm);
    }

    public static final Command setGamepieceCube(Arm arm) {
        return new InstantCommand(() -> arm.setGameObject(GameObjectType.CUBE), arm);
    }

    public static final Command waitForIntakeThenRetract(Arm arm) {
        return new SequentialCommandGroup(
            new WaitUntilCommand(arm::isDoneIntaking),
            new ArmSetGoalTillFinished(arm, GoalState.TRANSPORT)
        );
    }

    public static final Command waitForScoreThenRetract(Arm arm) {
        return new SequentialCommandGroup(
            new WaitUntilCommand(arm::isDoneScoring),
            new ArmSetGoalTillFinished(arm, GoalState.STOW)
        );
    }

    public static final Command waitForScoreThenPartialRetract(Arm arm) {
        return new SequentialCommandGroup(
            new WaitUntilCommand(arm::isDoneScoring),
            new ArmSetGoalTillFinished(arm, GoalState.SCORE_WAIT)
        );
    }

    public static final Command retract(Arm arm) {
        return new ConditionalCommand(
            new ArmSetGoalTillFinished(arm, GoalState.TRANSPORT), 
            new ArmSetGoalTillFinished(arm, GoalState.STOW), 
            arm::gripperHasGamepiece
        );
    }

    public static final Command groundIntakeOpen(Arm arm) { 
        return new ConditionalCommand(
            new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_GROUND), 
            new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_GROUND), 
            arm::gameObjectIsCone
        );
    }

    public static final Command shelfIntakeOpen(Arm arm, Drive drive) {
        return 
        // new ConditionalCommand(
            // new ConditionalCommand(
            //     new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_SHELF), 
            //     new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_SHELF), 
            //     arm::gameObjectIsCone
            // ), 
            new SequentialCommandGroup(
                // new ArmSetGoalTillFinished(arm, GoalState.INTAKE_WAIT_SHELF),
                // new WaitUntilCommand(drive::autoAlignAtTarget), // this should wait until we are close enough
                new ConditionalCommand(
                    new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_SHELF), 
                    new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_SHELF), 
                    arm::gameObjectIsCone
                )
            );//, 
            // drive::autoAlignAtTarget // this should also check if we are close enough
        // );
    }

    public static final Command autoIntakeShelf(Arm arm, Drive drive) {
        return new SequentialCommandGroup(
            shelfIntakeOpen(arm, drive),
            waitForIntakeThenRetract(arm)
        );
    }

    public static final Command autoScore(GoalState waitState, GoalState goalState, Arm arm, Drive drive) {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new ArmSetGoalTillFinished(arm, goalState),
                waitForScoreThenRetract(arm)
            ), 
            new SequentialCommandGroup(
                new ArmSetGoalTillFinished(arm, waitState),
                new WaitUntilCommand(drive::readyToScore),
                new ArmSetGoalTillFinished(arm, goalState),
                waitForScoreThenRetract(arm)
            ),
            drive::readyToScore
        );
    }

    public static final Command autoScore(GoalState waitState, GoalState stateIfCube, GoalState stateIfCone, Arm arm, Drive drive) {
        return new ConditionalCommand(
            autoScore(waitState, stateIfCone, arm, drive), 
            autoScore(waitState, stateIfCube, arm, drive), 
            arm::gameObjectIsCone
        );
    }

    public static final Command instantAutoScore(GoalState state, Arm arm) {
        return new SequentialCommandGroup(
            new ArmSetGoalTillFinished(arm, state),
            waitForScoreThenPartialRetract(arm)
        );
    }

    public static final Command setAlignStateOverride(boolean value, Drive drive) {
        return new InstantCommand(() -> drive.setAlignStateOverride(value));
    }

    public static final Command alignStateOverrideButton(Drive drive) {
        return new StartEndCommand(() -> drive.setAlignStateOverride(true), () -> drive.setAlignStateOverride(false));
    }

    public static final Command armFailureSwitch(Arm arm) {
        return new StartEndCommand(() -> arm.setForceFailure(true), () -> arm.setForceFailure(false));
    }
}
