// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.SupplierWidget;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.util.VirtualSubsystem;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LED.WantedAction;

public class Robot extends LoggedRobot {
    private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
    private static final double lowBatteryVoltage = 12.5;
    private static final double lowBatteryDisabledTime = 1.5;
    
    private RobotContainer robotContainer;
    private Command autoCommand;
    private double autoStart;
    private boolean autoMessagePrinted;
    private final Timer canErrorTimer = new Timer();
    private final Timer canErrorTimerInitial = new Timer();
    private final Timer disabledTimer = new Timer();

    private static final HashMap<String, Double> subsystemCurrents = new HashMap<>();
    public static final double BATTERY_NOMINAL_RESISTANCE_OHMS = 0.020;
    public static final double BATTERY_NOMINAL_VOLTAGE = 12.0;

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    private final PDPSim simPDH = new PDPSim(pdh);

    private final Alert logNoFileAlert =
    new Alert("No log path set for current robot. Data will NOT be logged.", AlertType.WARNING);
    private final Alert logReceiverQueueAlert =
    new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
    private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
    private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);
    
    public Robot() {
        super(Constants.loopPeriodSecs);
    }
    
    @Override
    public void robotInit() {
        System.out.println("Starting TechnoDogs 3707 Robot Code (2024, Codename Skynyrd)");
        
        // Record metadata
        Logger.recordMetadata("Robot", Constants.getRobot().toString());
        Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("Codename", "Skynyrd");
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
            Logger.recordMetadata("GitDirty", "All changes committed");
            break;
            case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
            default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
        }
        
        // Set up data receivers & replay source
        switch (Constants.getMode()) {
            case REAL:
            String folder = Constants.logFolders.get(Constants.getRobot());
            if (folder != null) {
                // Logger.addDataReceiver(new WPILOGWriter(folder)); TODO: fix this
            } else {
                logNoFileAlert.set(true);
            }
            Logger.addDataReceiver(new NT4Publisher());
            switch (Constants.getRobot()) {
                case ROBOT_2023_HEAVYMETAL:
                    LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
                    break;
                default:
                    break;
            }
            break;
            
            case SIM:
            Logger.addDataReceiver(new NT4Publisher());
            break;
            
            case REPLAY:
            String path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(path));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
            break;
        }
        
        // Start AdvantageKit logger
        setUseTiming(Constants.getMode() != Mode.REPLAY);
        Logger.start();
        
        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger
            .recordOutput(
            "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
        .onCommandInitialize(
        (Command command) -> {
            logCommandFunction.accept(command, true);
        });
        CommandScheduler.getInstance()
        .onCommandFinish(
        (Command command) -> {
            logCommandFunction.accept(command, false);
        });
        CommandScheduler.getInstance()
        .onCommandInterrupt(
        (Command command) -> {
            logCommandFunction.accept(command, false);
        });
        
        robotContainer = new RobotContainer(this);
    }
    
    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        VirtualSubsystem.periodicAll();
        CommandScheduler.getInstance().run();

        // Check logging fault
        logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

        // Robot container periodic methods
        // robotContainer.updateDemoControls();
        robotContainer.checkControllers();
        // robotContainer.updateHPModeLeds();

        if (Constants.getMode() == Mode.SIM) {
            RoboRioSim.setVInVoltage(getSimulatedVoltage());
            simPDH.setVoltage(getSimulatedVoltage());
            simPDH.setCurrent(0, getSimulatedCurrent());
            Logger.recordOutput("SimData/EstimatedBatteryCurrent", getSimulatedCurrent());
            Logger.recordOutput("SimData/EstimatedBatteryVoltage", getSimulatedVoltage());
        }

        // Update CAN error alert
        var canStatus = RobotController.getCANStatus();
        if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
        canErrorTimer.reset();
        }
        canErrorAlert.set(
            !canErrorTimer.hasElapsed(canErrorTimeThreshold)
                && canErrorTimerInitial.hasElapsed(canErrorTimeThreshold));

        // Update low battery alert
        if (DriverStation.isEnabled()) {
            disabledTimer.reset();
        }

        // Log list of NT clients
        List<String> clientNames = new ArrayList<>();
        List<String> clientAddresses = new ArrayList<>();
        for (var client : NetworkTableInstance.getDefault().getConnections()) {
            clientNames.add(client.remote_id);
            clientAddresses.add(client.remote_ip);
        }
        Logger
            .recordOutput("NTClients/Names", clientNames.toArray(new String[clientNames.size()]));
        Logger
            .recordOutput(
                "NTClients/Addresses", clientAddresses.toArray(new String[clientAddresses.size()]));

        // Print auto duration
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    System.out.println(
                        String.format(
                            "*** Auto finished in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
                } else {
                    System.out.println(
                        String.format(
                            "*** Auto cancelled in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
                }
                autoMessagePrinted = true;
            }
        }

        SupplierWidget.updateAll();
        Threads.setCurrentThreadPriority(true, 10);
    }
    
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {
        if (RobotController.getBatteryVoltage() < lowBatteryVoltage
            && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
            LED.setWantedAction(WantedAction.DISPLAY_BATTERY_LOW);
            lowBatteryAlert.set(true);
        } else if (robotContainer.hasConfigErrors()) {
            LED.setWantedAction(WantedAction.DISPLAY_CONFIGURE_FAULT);
            lowBatteryAlert.set(false);
        } else {
            LED.setWantedAction(WantedAction.DISPLAY_GOOD_BATTERY);
            lowBatteryAlert.set(false);
        }
    }
    
    @Override
    public void disabledExit() {}
    
    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutonomousCommand();
        autoStart = Timer.getFPGATimestamp();
        autoCommand.schedule();
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void autonomousExit() {}
    
    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
        LED.setWantedAction(WantedAction.DISPLAY_ARM);
        robotContainer.onTeleopInit();
    }
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void teleopExit() {}
    
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override
    public void testPeriodic() {}
    
    @Override
    public void testExit() {}

    public static void updateSimCurrentDraw(String subsystem, double totalCurrent) {
        if (subsystemCurrents.putIfAbsent(subsystem, totalCurrent) != null) {
            subsystemCurrents.replace(subsystem, totalCurrent);
        }
    }

    public static double getSimulatedCurrent() {
        double current = 0.0;

        for (double l : subsystemCurrents.values()) {
            current += l;
        }

        return current;
    }

    public static double getSimulatedVoltage() {
        return BatterySim.calculateLoadedBatteryVoltage(BATTERY_NOMINAL_VOLTAGE, BATTERY_NOMINAL_RESISTANCE_OHMS, getSimulatedCurrent());
    }
}
