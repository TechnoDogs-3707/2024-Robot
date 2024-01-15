// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.ArmCommandFactory;
import frc.robot.commands.ArmSetGoalTillFinished;
import frc.robot.commands.AutonXModeCommand;
import frc.robot.commands.DriveAutoAlignCommand;
import frc.robot.commands.DriveUtilityCommandFactory;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.XModeDriveCommand;
import frc.robot.lib.OverrideSwitches;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSimV1;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.controllerFeedback.ControllerFeedback;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.FalconSwerveIO;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroNavXIO;
import frc.robot.subsystems.drive.GyroPigeonIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerStateMachine;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIOCANdle;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.localizer.Localizer;
import frc.robot.subsystems.localizer.LocalizerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOFalcon;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterStateMachine;

public class RobotContainer {
    private Drive drive;
    private Arm arm;
    private Shooter shooter;
    private Indexer indexer;
    private LED leds;
    private Localizer vision;
    private ControllerFeedback controllerFeedback;
    private Dashboard dashboard;

    public final LoggedDashboardChooser<Command> autoChooser;

    // DRIVER CONTROLS
    private final CommandXboxController driver = new CommandXboxController(0);

    private final Trigger driverSlowMode = driver.leftBumper();
    private final Trigger driverXMode = driver.x();
    private final Trigger driverGyroReset = driver.back().debounce(1, DebounceType.kRising); // delay gyro reset for 1 second
    private final Trigger driverAutoAlign = driver.rightTrigger(0.1);
    // private final Trigger driverSnapClosestCardinal = driver.rightTrigger(0.2);
    // private final Trigger driverSnapOppositeCardinal = driver.leftTrigger(0.2);
    private final Trigger driverTempDisableFieldOriented = driver.rightBumper();
    
    private final Trigger testModeStow = driver.b();
    private final Trigger testModeIntake = driver.leftTrigger(0.1);
    private final Trigger testModeScore = driver.y();
    private final Trigger testModeAmp = driver.povUp();

    // OPERATOR CONTROLS
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Trigger operatorResetMotionPlanner = operator.back().debounce(1, DebounceType.kRising);
    // private final Trigger operatorOverrideScore = operator.b();

    // OVERRIDE SWITCHES
    private final OverrideSwitches overrides = new OverrideSwitches(5);

    // private final Trigger driverResetAngle = overrides.driverSwitch(0).debounce(1, DebounceType.kRising); // Reset gyro angle to forwards
    private final Trigger driverGyroFail = overrides.driverSwitch(0); // Ingore sensor readings from gyro
    private final Trigger driverReseedPosition = overrides.driverSwitch(1).debounce(1, DebounceType.kRising); // Gather avereage position from vision and update
    private final Trigger driverAssistFail = overrides.driverSwitch(2); // disable all drive assists

    // private final Trigger armForceEnable = overrides.operatorSwitch(0); // bypass arm sanity checks and force manual control
    // private final Trigger armHomingSequence = overrides.operatorSwitch(1).debounce(1, DebounceType.kRising); // run the arm calibration sequence
    // private final Trigger overrideArmSafety = overrides.operatorSwitch(2); // run arm at full speed even off FMS
    // private final Trigger overrideLedBrightness = overrides.operatorSwitch(3); // full led brightness when off FMS
    private final Trigger ledsIndicateFailed = overrides.operatorSwitch(4); // indicate arm failed on LEDS

    // Virtual Triggers
    private final Trigger driverNoFieldOriented = driverTempDisableFieldOriented.or(driverGyroFail);
    private final Trigger robotTeleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
    
    public final LoggedDashboardNumber endgameAlert1 = new LoggedDashboardNumber("Endgame Alert #1", 30.0);
    public final LoggedDashboardNumber endgameAlert2 = new LoggedDashboardNumber("Endgame Alert #2", 15.0);

    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.WARNING);
    private final Alert overrideDisconnected = new Alert("Override controller disconnected (port 5).", AlertType.INFO);

    public RobotContainer(Robot robot) {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_2023_HEAVYMETAL:
                    drive = new Drive(
                            // new GyroNavXIO(SPI.Port.kMXP),
                            // new GyroPigeon5IO(9, "canivore"),
                            new GyroPigeonIO(9, "canivore"),
                            new FalconSwerveIO(0, "canivore"),
                            new FalconSwerveIO(1, "canivore"),
                            new FalconSwerveIO(2, "canivore"),
                            new FalconSwerveIO(3, "canivore"));
                    // arm = new Arm(new ArmIOFalcons(), new GripperIOFalcon());
                    arm = new Arm(new ArmIOSimV1());
                    leds = new LED(new LEDIOCANdle(8, "canivore"));
                    break;
                case ROBOT_2023_FLAPJACK:
                    drive = new Drive(
                            new GyroNavXIO(Port.kMXP),
                            new FalconSwerveIO(0, "canivore"), 
                            new FalconSwerveIO(1, "canivore"), 
                            new FalconSwerveIO(2, "canivore"), 
                            new FalconSwerveIO(3, "canivore"));
                    // arm = new Arm(new ArmIOSimV1(), new GripperMiniNeoSimIO()); // simulate arm on chassis bot
                    // leds =
                    // vision = new Localizer(new LocalizerIOLL3(), drive::addVisionPose);
                    break;
                case ROBOT_SIMBOT:
                    drive = new Drive(
                            new GyroIO() {}, // Empty gyro object defaults to wheel delta integration
                            new SimSwerveIO(),
                            new SimSwerveIO(),
                            new SimSwerveIO(),
                            new SimSwerveIO());
                    arm = new Arm(new ArmIOSimV1());
                    // indexer = new Indexer(null)
                    shooter = new Shooter(new ShooterIOSim());
                    leds = new LED(new LEDIOSim(127));
                    break;
                default:
                    throw new IllegalStateException("Selected robot is not valid.");
            }
        }

        if (drive == null) {
            drive = new Drive(
                    new GyroIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    });
        }

        if (arm == null) {
            arm = new Arm(new ArmIO() {

            });
        }

        if (shooter == null) {
            shooter = new Shooter(new ShooterIO() {
                
            });
        }

        if (indexer == null) {
            indexer = new Indexer(new IndexerIO() {

            });
        }

        if (leds == null) {
            leds = new LED(new LEDIO() {
                
            });
        }

        if (vision == null) {
            vision = new Localizer(new LocalizerIO() {}, (v) -> {});
        }

        controllerFeedback = new ControllerFeedback(driver.getHID(), operator.getHID());

        if (Constants.tuningMode) {
            new Alert("Tuning mode active! This should not be used in competition.", AlertType.INFO).set(true);
        }

        // Register Commands with PathPlanner
        NamedCommands.registerCommand("Drive Set X Mode", new AutonXModeCommand(drive));
        NamedCommands.registerCommand("Enable Align Override", ArmCommandFactory.setAlignStateOverride(true, drive));
        NamedCommands.registerCommand("Disable Align Override", ArmCommandFactory.setAlignStateOverride(false, drive));
        drive.setupPathPlanner();

        autoChooser = new LoggedDashboardChooser<>("autonMode", AutoBuilder.buildAutoChooser());

        dashboard = new Dashboard(robot, this, drive, arm, leds, vision, controllerFeedback);
        dashboard.resetWidgets();

        configureBindings();
        setDefaultCommands();
    }

    public void checkControllers() {
        driverDisconnected.set(
                !DriverStation.isJoystickConnected(driver.getHID().getPort())
                        || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
        operatorDisconnected.set(
                !DriverStation.isJoystickConnected(operator.getHID().getPort())
                        || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
        overrideDisconnected.set(!overrides.isConnected());
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
            new DriveWithController(
                drive, 
                this::getDriveInputs, 
                driverSlowMode::getAsBoolean, 
                driverNoFieldOriented::getAsBoolean, 
                () -> false,// driverSnapClosestCardinal::getAsBoolean,
                () -> false// driverSnapOppositeCardinal::getAsBoolean
            )
        );
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Drive button bindings
        driverXMode.whileTrue(new XModeDriveCommand(drive));
        driverGyroReset.onTrue(DriveUtilityCommandFactory.resetGyro(drive));
        driverAutoAlign.whileTrue(new DriveAutoAlignCommand(drive, arm));

        testModeStow.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> arm.setGoalState(GoalState.STOW), arm),
            new InstantCommand(() -> indexer.setWantedAction(IndexerStateMachine.WantedAction.IDLE), indexer),
            new InstantCommand(() -> shooter.setWantedAction(ShooterStateMachine.WantedAction.IDLE), shooter)
        ));

        testModeIntake.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> arm.setGoalState(GoalState.INTAKE_SOURCE), arm),
            new InstantCommand(() -> indexer.setWantedAction(IndexerStateMachine.WantedAction.INTAKE), indexer),
            new InstantCommand(() -> shooter.setWantedAction(ShooterStateMachine.WantedAction.OFF), shooter)
        ));

        testModeScore.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> arm.setGoalState(GoalState.SCORE_SPEAKER_SUBWOOFER), arm),
            new InstantCommand(() -> indexer.setWantedAction(IndexerStateMachine.WantedAction.SCORE), indexer),
            new InstantCommand(() -> shooter.setWantedAction(ShooterStateMachine.WantedAction.SHOOT), shooter),
            new InstantCommand(() -> shooter.setSetpointSpeedLeft(150), shooter),
            new InstantCommand(() -> shooter.setSetpointSpeedRight(150), shooter)
        ));

        testModeAmp.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> arm.setGoalState(GoalState.SCORE_AMP), arm),
            new InstantCommand(() -> indexer.setWantedAction(IndexerStateMachine.WantedAction.SCORE), indexer),
            new InstantCommand(() -> shooter.setWantedAction(ShooterStateMachine.WantedAction.SHOOT), shooter)
        ));


        // Driver override switches
        driverReseedPosition.onTrue(DriveUtilityCommandFactory.reseedPosition(drive));
        driverGyroFail.onTrue(DriveUtilityCommandFactory.failGyro(drive));
        driverGyroFail.onFalse(DriveUtilityCommandFactory.unFailGyro(drive));
        driverAssistFail.onTrue(DriveUtilityCommandFactory.failDriveAssist(drive));
        driverAssistFail.onFalse(DriveUtilityCommandFactory.unFailDriveAssist(drive));

        //Operator button bindings
        
        operatorResetMotionPlanner.onTrue(new InstantCommand(() -> arm.setResetMotionPlanner(true), arm));
        operatorResetMotionPlanner.onFalse(new InstantCommand(() -> arm.setResetMotionPlanner(false), arm));

        // operatorOverrideScore.and(robotTeleopEnabled).whileTrue(ArmCommandFactory.alignStateOverrideButton(drive));

        // Operator override switches
        // armForceEnable
        // armHomingSequence
        // overrideArmSafety
        ledsIndicateFailed.whileTrue(ArmCommandFactory.armFailureSwitch(arm));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private ControllerDriveInputs getDriveInputs() {
        return new ControllerDriveInputs(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX()).applyDeadZone(0.05, 0.05, 0.05, 0.08).cubeInputs();
    }

    protected void onTeleopInit() {
        drive.setAlignStateOverride(false);
    }

    public boolean hasConfigErrors() {
        return false;
    }
}
