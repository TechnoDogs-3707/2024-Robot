package frc.robot.lib.faultReporting;

import frc.robot.lib.dashboard.Alert.AlertType;

public class ReportedFault {
    private final String mSource;
    private final String mMessage;
    private final AlertType mSeverity;

    public ReportedFault(String source, String message, AlertType severity) {
        mSource = source;
        mMessage = message;
        mSeverity = severity;
    }

    public String getSource() {
        return mSource;
    }

    public String getMessage() {
        return mMessage;
    }

    public AlertType getSeverity() {
        return mSeverity;
    }
}
