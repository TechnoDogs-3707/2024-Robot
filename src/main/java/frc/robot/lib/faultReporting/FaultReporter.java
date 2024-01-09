package frc.robot.lib.faultReporting;

import java.util.ArrayList;
import java.util.Optional;

public interface FaultReporter {
    public Optional<ArrayList<ReportedFault>> getFaults();
}
