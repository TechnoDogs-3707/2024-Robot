package frc.robot.lib.faultReporting;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.lib.dashboard.Alert.AlertType;

public class TalonFXFaultReporter implements FaultReporter{
    protected static final ArrayList<TalonFXFaultReporter> faultReporters = new ArrayList<>();

    private final TalonFX mTalon;

    private final ArrayList<StatusSignal<Boolean>> mErrorFaults;

    public TalonFXFaultReporter(TalonFX talon) {
        mTalon = talon;

        mErrorFaults = new ArrayList<>();
        mErrorFaults.add(mTalon.getFault_BridgeBrownout());
        mErrorFaults.add(mTalon.getFault_DeviceTemp());
        mErrorFaults.add(mTalon.getFault_FusedSensorOutOfSync());
        mErrorFaults.add(mTalon.getFault_Hardware());
        mErrorFaults.add(mTalon.getFault_MissingDifferentialFX());
        mErrorFaults.add(mTalon.getFault_OverSupplyV());
        mErrorFaults.add(mTalon.getFault_ProcTemp());
        mErrorFaults.add(mTalon.getFault_RemoteSensorDataInvalid());
        mErrorFaults.add(mTalon.getFault_RemoteSensorPosOverflow());
        mErrorFaults.add(mTalon.getFault_RemoteSensorReset());
        mErrorFaults.add(mTalon.getFault_Undervoltage());
        mErrorFaults.add(mTalon.getFault_UnlicensedFeatureInUse());
        mErrorFaults.add(mTalon.getFault_UnstableSupplyV());
        mErrorFaults.add(mTalon.getFault_UsingFusedCANcoderWhileUnlicensed());

        faultReporters.add(this);
    }

    @Override
    public Optional<ArrayList<ReportedFault>> getFaults() {
        boolean hasAnyFaults = false;

        for (StatusSignal<Boolean> statusSignal : mErrorFaults) {
            statusSignal.refresh();
            hasAnyFaults |= statusSignal.getValue();
        }

        ArrayList<ReportedFault> list = new ArrayList<>();
        if (hasAnyFaults) {
            list.add(new ReportedFault("TalonFX ID: " + mTalon.getDeviceID(), "Motor Controller Fault Detected! Check in Phoenix Tuner", AlertType.ERROR));
        }

        return (list.size() > 0 ? Optional.of(list) : Optional.empty());
    }

    
}
