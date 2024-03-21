package frc.robot.lib.phoenixpro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class PhoenixErrorChecker {

    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkErrorV5(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + " " + errorCode, false);
        }
    }

    public static boolean checkErrorAndRetryV5(Supplier<ErrorCode> errorCode, int numTries) {
        ErrorCode code = errorCode.get();
        int tries = 0;
        while(code != ErrorCode.OK && tries < numTries) {
            DriverStation.reportWarning("Retrying CTRE Phoenix v5 Device Config " + code.name(), false);
            code = errorCode.get();
            tries++;
        }
        if (code != ErrorCode.OK) {
            DriverStation.reportError("Failed to execute phoenix v5 api call after " + numTries + " attempts", false);
            return false;
        }
        return true;
    }

    public static boolean checkErrorAndRetryV5(Supplier<ErrorCode> errorCode) {
        return checkErrorAndRetryV5(errorCode, 5);
    }

    public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
        StatusCode code = function.get();
        int tries = 0;
        while (code != StatusCode.OK && tries < numTries) {
            DriverStation.reportWarning("Retrying CTRE Phoenix 6 Device Config " + code.getName(), false);
            code = function.get();
            tries++;
        }
        if (code != StatusCode.OK) {
            DriverStation.reportError("Failed to execute phoenix 6 api call after " + numTries + " attempts", false);
            return false;
        }
        return true;
    }



    /**
     * checks the specified error code and throws an exception if there are any issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkErrorWithThrow(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(message + " " + errorCode);
        }
    }

    public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
        return checkErrorAndRetry(function, 5);
    }
}