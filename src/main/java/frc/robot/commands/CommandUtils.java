// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** Add your docs here. */
public class CommandUtils{

    public static Command SkipIfTrueOrWaitCommand(BooleanSupplier condition, Command doOnceMet, Command doIfNotMet) {
        return new ConditionalCommand(
            doOnceMet, 
            new SequentialCommandGroup(
                doIfNotMet,
                new WaitUntilCommand(condition),
                doOnceMet
            ),
            condition
        );
    }
}
