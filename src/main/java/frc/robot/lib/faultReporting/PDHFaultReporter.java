package frc.robot.lib.faultReporting;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.lib.dashboard.Alert.AlertType;

public class PDHFaultReporter implements FaultReporter {
    protected static PDHFaultReporter mInstance;
    private final PowerDistribution mPDH;

    PDHFaultReporter(PowerDistribution PDH) {
        mPDH = PDH;

        mInstance = this;
    }

    @Override
    public Optional<ArrayList<ReportedFault>> getFaults() {
        boolean hasMainBreakerFaults = false;
        boolean hasAuxBreakerFaults = false;
        boolean hasSystemFaults = false;

        PowerDistributionFaults faults = mPDH.getFaults();
        // TODO: somehow move these, or comment out channels that aren't used
        hasMainBreakerFaults |= faults.Channel0BreakerFault;
        hasMainBreakerFaults |= faults.Channel1BreakerFault;
        hasMainBreakerFaults |= faults.Channel2BreakerFault;
        hasMainBreakerFaults |= faults.Channel3BreakerFault;
        hasMainBreakerFaults |= faults.Channel4BreakerFault;
        hasMainBreakerFaults |= faults.Channel5BreakerFault;
        hasMainBreakerFaults |= faults.Channel6BreakerFault;
        hasMainBreakerFaults |= faults.Channel7BreakerFault;
        hasMainBreakerFaults |= faults.Channel8BreakerFault;
        hasMainBreakerFaults |= faults.Channel9BreakerFault;
        hasMainBreakerFaults |= faults.Channel10BreakerFault;
        hasMainBreakerFaults |= faults.Channel11BreakerFault;
        hasMainBreakerFaults |= faults.Channel12BreakerFault;
        hasMainBreakerFaults |= faults.Channel13BreakerFault;
        hasMainBreakerFaults |= faults.Channel14BreakerFault;
        hasMainBreakerFaults |= faults.Channel15BreakerFault;
        hasMainBreakerFaults |= faults.Channel16BreakerFault;
        hasMainBreakerFaults |= faults.Channel17BreakerFault;
        hasMainBreakerFaults |= faults.Channel18BreakerFault;
        hasMainBreakerFaults |= faults.Channel19BreakerFault;

        hasAuxBreakerFaults |= faults.Channel20BreakerFault;
        hasAuxBreakerFaults |= faults.Channel21BreakerFault;
        hasAuxBreakerFaults |= faults.Channel22BreakerFault;
        hasAuxBreakerFaults |= faults.Channel23BreakerFault;

        hasSystemFaults |= faults.Brownout;
        hasSystemFaults |= faults.CanWarning;
        hasSystemFaults |= faults.HardwareFault;

        if (hasMainBreakerFaults || hasAuxBreakerFaults || hasSystemFaults) {
            ArrayList<ReportedFault> list = new ArrayList<>();
            if (hasMainBreakerFaults) {
                list.add(new ReportedFault("PDH ID: " + mPDH.getModule(), "Main (High Current) Breaker Fault Detected", AlertType.WARNING));
            }
            if (hasAuxBreakerFaults) {
                list.add(new ReportedFault("PDH ID: " + mPDH.getModule(), "Auxillary (Low Current) Breaker Fault Detected", AlertType.WARNING));
            }
            if (hasSystemFaults) {
                list.add(new ReportedFault("PDH ID: " + mPDH.getModule(), "System Fault Detected", AlertType.ERROR));
            }
            return Optional.of(list);
        } else {
            return Optional.empty();
        }
    }

    
}
