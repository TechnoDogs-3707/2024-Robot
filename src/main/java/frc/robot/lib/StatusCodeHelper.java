// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class StatusCodeHelper {
    public static StatusCode CheckStatusCode(final StatusCode code) {
        final boolean printWarning = code.isWarning();
        final boolean printError = code.isError();

        if (printError) {
            DriverStation.reportError(buildMessage(code, StatusCodeResponseType.kError), true);
        } else if (printWarning) {
            DriverStation.reportWarning(buildMessage(code, StatusCodeResponseType.kWarning), true);
        }

        return code; // pass StatusCode through
    }

    private enum StatusCodeResponseType {
        kWarning("Warning"),
        kError("Error"),;

        public final String title;

        StatusCodeResponseType(String title) {
            this.title = title;
        }
    }

    private static String buildMessage(final StatusCode code, final StatusCodeResponseType type) {
        return "[StatusCodeHelper] " 
        + type.title
        + ": "
        + code.getName() 
        + " Description: "
        + code.getDescription();
    }
}
