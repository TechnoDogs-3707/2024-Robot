// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.AutoScoreShooterAmp;
import frc.robot.commands.AutoScoreShooterPodium;
import frc.robot.commands.AutoScoreShooterSubwoofer;
import frc.robot.commands.AutonXModeCommand;
import frc.robot.commands.DriveUtilityCommandFactory;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.IndexerJamClearing;
import frc.robot.commands.IndexerReset;
import frc.robot.commands.IntakeNoteGroundToIndexer;
import frc.robot.commands.ShooterPrepare;
import frc.robot.commands.XModeDriveCommand;
import frc.robot.commands.climb.ClimbAutoRaise;
import frc.robot.commands.climb.ClimbManualOverride;
import frc.robot.commands.climb.ClimbPoweredRetract;
import frc.robot.commands.climb.ClimbReset;
import frc.robot.lib.OverrideSwitches;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.controllerFeedback.ControllerFeedback;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveIOTalonFX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroNavXIO;
import frc.robot.subsystems.drive.GyroPigeonIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeployIO;
import frc.robot.subsystems.intakeDeploy.IntakeDeployIOSim;
import frc.robot.subsystems.intakeDeploy.IntakeDeployIOTalonFX;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIOCANdle;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.localizer.Localizer;
import frc.robot.subsystems.localizer.LocalizerIO;
import frc.robot.subsystems.localizer.LocalizerIOLL3;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.TiltIO;
import frc.robot.subsystems.tilt.TiltIOSim;
import frc.robot.subsystems.tilt.TiltIOTalonFX;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class RobotContainer {
    private Drive drive;
    private IntakeDeploy intakeDeploy;
    private Intake intake;
    private Flywheels flywheels;
    private Tilt tilt;
    private Indexer indexer;
    private Climb climb;
    private LED leds;
    private Localizer vision;
    private ObjectiveTracker objective;
    private ControllerFeedback controllerFeedback;
    private Dashboard dashboard;

    public final LoggedDashboardChooser<Command> autoChooser;

    // DRIVER CONTROLS
    private final CommandPS5Controller driver = new CommandPS5Controller(0);

    private final Trigger driverSlowMode = driver.L1();
    private final Trigger driverXMode = driver.cross();
    private final Trigger driverGyroReset = driver.create().debounce(0.5, DebounceType.kRising); // delay gyro reset for 1 second
    private final Trigger driverAlignAmp = driver.triangle();
    private final Trigger driverJamClear = driver.povDown();
    // private final Trigger driverAutoAlignClosest = driver.L2();
    // private final Trigger driverAutoAlignClosest = driver.PS();
    private final Trigger driverDeployIntake = driver.L2();
    private final Trigger driverAlignPodium = driver.square();
    // private final Trigger driverSnapAngleIgnoringPreference = driver.circle();
    private final Trigger driverCancelAction = driver.circle();
    private final Trigger driverAutoShoot = driver.R2();
    // private final Trigger driverSnapOppositeCardinal = driver.leftTrigger(0.2);
    private final Trigger driverTempDisableFieldOriented = driver.R1();

    // OPERATOR CONTROLS
    private final CommandPS5Controller operator = new CommandPS5Controller(1);

    // private final Trigger operatorResetMotionPlanner = operator.back().debounce(1, DebounceType.kRising);
    private final Trigger operatorResetMotionPlanner = operator.create().debounce(0.5, DebounceType.kRising);
    private final Trigger operatorCancelAction = operator.R1();
    private final Trigger operatorClimbShift = operator.L1();

    private final Trigger operatorScoreOverride = operator.circle();

    private final Trigger operatorSubwoofer = operatorClimbShift.negate().and(operator.povRight());
    private final Trigger operatorPodium = operatorClimbShift.negate().and(operator.povLeft());
    private final Trigger operatorAmp = operatorClimbShift.negate().and(operator.povUp());
    private final Trigger operatorJamClear = operatorClimbShift.negate().and(operator.povDown());

    private final Trigger operatorClimbRaise = operatorClimbShift.and(operator.povUp());
    private final Trigger operatorClimbPull = operatorClimbShift.and(operator.povDown());
    private final Trigger operatorClimbReset = operatorClimbShift.and(operator.povLeft());
    private final Trigger operatorClimbManual = operatorClimbShift.and(operator.povRight());

    private final Supplier<Double> operatorClimbThrottle = () -> -Util.handleDeadband(operator.getLeftY(), 0.05);

    // OVERRIDE SWITCHES
    private final OverrideSwitches overrides = new OverrideSwitches(5);

    // private final Trigger driverResetAngle = overrides.driverSwitch(0).debounce(1, DebounceType.kRising); // Reset gyro angle to forwards
    private final Trigger driverGyroFail = overrides.driverSwitch(0); // Ingore sensor readings from gyro
    private final Trigger driverReseedPosition = overrides.driverSwitch(1).debounce(1, DebounceType.kRising); // Gather average position from vision and update
    private final Trigger driverAssistFail = overrides.driverSwitch(2); // disable all drive assists

    // private final Trigger armForceEnable = overrides.operatorSwitch(0); // bypass arm sanity checks and force manual control
    // private final Trigger armHomingSequence = overrides.operatorSwitch(1).debounce(1, DebounceType.kRising); // run the arm calibration sequence
    // private final Trigger overrideArmSafety = overrides.operatorSwitch(2); // run arm at full speed even off FMS
    // private final Trigger overrideLedBrightness = overrides.operatorSwitch(3); // full led brightness when off FMS
    // private final Trigger ledsIndicateFailed = overrides.operatorSwitch(4); // indicate arm failed on LEDS

    // Virtual Triggers
    private final Trigger driverNoFieldOriented = driverTempDisableFieldOriented.or(driverGyroFail);
    
    public final LoggedDashboardNumber endgameAlert1 = new LoggedDashboardNumber("Endgame Alert #1", 30.0);
    public final LoggedDashboardNumber endgameAlert2 = new LoggedDashboardNumber("Endgame Alert #2", 15.0);

    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.WARNING);
    private final Alert overrideDisconnected = new Alert("Override controller disconnected (port 5).", AlertType.INFO);

    public RobotContainer(Robot robot) {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_2024_SONIC:
                    drive = new Drive(
                        new GyroPigeonIO(9, "rio"), 
                        new SwerveIOTalonFX(0, "canivore"), 
                        new SwerveIOTalonFX(1, "canivore"), 
                        new SwerveIOTalonFX(2, "canivore"), 
                        new SwerveIOTalonFX(3, "canivore")
                    );
                    intakeDeploy = new IntakeDeploy(new IntakeDeployIOTalonFX());
                    intake = new Intake(new IntakeIOTalonFX());
                    flywheels = new Flywheels(new FlywheelsIOTalonFX());
                    tilt = new Tilt(new TiltIOTalonFX());
                    indexer = new Indexer(new IndexerIOTalonFX());
                    climb = new Climb(new ClimbIOTalonFX());
                    leds = new LED(new LEDIOSim(127));
                    // vision = new Localizer(new LocalizerIOLL3(), drive::addVisionPose);
                    break;
                // case ROBOT_2024_HARD_ROCK:
                //     drive = new Drive(
                //         new GyroPigeonIO(9, "canivore"), 
                //         new SwerveIOTalonFX(0, "canivore"), 
                //         new SwerveIOTalonFX(1, "canivore"), 
                //         new SwerveIOTalonFX(2, "canivore"), 
                //         new SwerveIOTalonFX(3, "canivore")
                //     );
                //     arm = new Arm(new ArmIOSimV1());
                //     intake = new Intake(new IntakeIOSim());
                //     shooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIOSim());
                //     shooterTilt = new ShooterTilt(new ShooterTiltIOSim());
                //     indexer = new Indexer(new IndexerIOSim());
                //     leds = new LED(new LEDIOSim(127));
                //     vision = new Localizer(new LocalizerIOLL3(), drive::addVisionPose);
                //     break;
                case ROBOT_2023_HEAVYMETAL:
                    drive = new Drive(
                            // new GyroNavXIO(SPI.Port.kMXP),
                            // new GyroPigeon5IO(9, "canivore"),
                            new GyroPigeonIO(9, "canivore"),
                            new SwerveIOTalonFX(0, "canivore"),
                            new SwerveIOTalonFX(1, "canivore"),
                            new SwerveIOTalonFX(2, "canivore"),
                            new SwerveIOTalonFX(3, "canivore"));
                    // arm = new Arm(new ArmIOFalcons(), new GripperIOFalcon());
                    // arm = new Arm(new ArmIOSimV1());
                    leds = new LED(new LEDIOCANdle(8, "canivore"));
                    break;
                case ROBOT_2023_FLAPJACK:
                    drive = new Drive(
                            new GyroNavXIO(Port.kMXP),
                            new SwerveIOTalonFX(0, "canivore"), 
                            new SwerveIOTalonFX(1, "canivore"), 
                            new SwerveIOTalonFX(2, "canivore"), 
                            new SwerveIOTalonFX(3, "canivore"));
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
                    intakeDeploy = new IntakeDeploy(new IntakeDeployIOSim());
                    intake = new Intake(new IntakeIOSim());
                    indexer = new Indexer(new IndexerIOSim());
                    flywheels = new Flywheels(new FlywheelsIOSim());
                    tilt = new Tilt(new TiltIOSim());
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

        if (intakeDeploy == null) {
            intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {

            });
        }

        if (intake == null) {
            intake = new Intake(new IntakeIO() {
                
            });
        }

        if (flywheels == null) {
            flywheels = new Flywheels(new FlywheelsIO() {
                
            });
        }

        if (tilt == null) {
            tilt = new Tilt(new TiltIO() {
                
            });
        }

        if (indexer == null) {
            indexer = new Indexer(new IndexerIO() {

            });
        }

        if (climb == null) {
            climb = new Climb(new ClimbIO() {
                
            });
        }

        if (leds == null) {
            leds = new LED(new LEDIO() {
                
            });
        }

        if (vision == null) {
            vision = new Localizer(new LocalizerIO() {}, (v) -> {});
        }

        objective = new ObjectiveTracker(intake, indexer);

        controllerFeedback = new ControllerFeedback(driver.getHID(), operator.getHID());

        if (Constants.tuningMode) {
            new Alert("Tuning mode active! This should not be used in competition.", AlertType.INFO).set(true);
        }

        // Register Commands with PathPlanner
        NamedCommands.registerCommand("Drive Set X Mode", new AutonXModeCommand(drive));
        NamedCommands.registerCommand("Indexer Set Intake", indexer.setActionCommand(IndexerWantedAction.INTAKE));
        NamedCommands.registerCommand("Indexer Set Score", indexer.setActionCommand(IndexerWantedAction.SCORE));
        NamedCommands.registerCommand("Indexer Set Off", indexer.setActionCommand(IndexerWantedAction.OFF));
        NamedCommands.registerCommand("Flywheels Set Speed To 60", flywheels.setSpeedCommand(() -> 60.0));
        NamedCommands.registerCommand("Flywheels Set Shoot", flywheels.setActionCommand(FlywheelsWantedAction.SHOOT));
        NamedCommands.registerCommand("Flywheels Set Off", flywheels.setActionCommand(FlywheelsWantedAction.OFF));
        NamedCommands.registerCommand("Tilt Set Close", tilt.setGoalCommand(TiltGoalState.CLOSE));
        NamedCommands.registerCommand("Tilt Set Stow", tilt.setGoalCommand(TiltGoalState.STOW));
        NamedCommands.registerCommand("Intake Set Run", intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT));
        NamedCommands.registerCommand("Intake Set Off", intake.setActionCommand(IntakeWantedAction.OFF));
        //TODO: fix these broken commands
        // NamedCommands.registerCommand("Deploy Intake", arm.setGoalCommand(GoalState.INTAKE_GROUND));
        // NamedCommands.registerCommand("Stow Intake", arm.setGoalCommand(GoalState.STOW));
        drive.setupPathPlanner();

        autoChooser = new LoggedDashboardChooser<>("autonMode", AutoBuilder.buildAutoChooser());

        dashboard = new Dashboard(robot, this, drive, intakeDeploy, intake, flywheels, tilt, indexer, leds, vision, objective, controllerFeedback);
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
                driverSlowMode, 
                driverNoFieldOriented, 
                driverAlignPodium,
                driverAlignAmp
            )
        );

        flywheels.setDefaultCommand(new ShooterPrepare(indexer, flywheels));
        // shooterFlywheels.setDefaultCommand(new ShooterAutomaticCommand(shooterFlywheels, indexer));
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Drive button bindings
        driverXMode.whileTrue(new XModeDriveCommand(drive));
        driverGyroReset.onTrue(DriveUtilityCommandFactory.resetGyro(drive));
        // driverAutoAlignClosest.whileTrue(new DriveAutoAlignCommand(drive, objective, () -> true));
        // driverAlignAmp.whileTrue(new DriveAutoAlignCommand(drive, objective, () -> false));

        // Driver override switches
        driverReseedPosition.onTrue(DriveUtilityCommandFactory.reseedPosition(drive));
        driverGyroFail.onTrue(DriveUtilityCommandFactory.failGyro(drive));
        driverGyroFail.onFalse(DriveUtilityCommandFactory.unFailGyro(drive));
        driverAssistFail.onTrue(DriveUtilityCommandFactory.failDriveAssist(drive));
        driverAssistFail.onFalse(DriveUtilityCommandFactory.unFailDriveAssist(drive));

        driverCancelAction.or(operatorCancelAction).onTrue(new IntakeStow(arm, intake).alongWith(new IndexerReset(indexer, tilt, flywheels)));
        driverDeployIntake.onTrue(new IntakeNoteGroundToIndexer(arm, intake, indexer, flywheels, objective));

        //Operator button bindings
        // operatorIntakeSourceToHold.onTrue(new IntakeNoteSource(drive, arm, intake, objective));
        operatorSubwoofer.onTrue(new AutoScoreShooterSubwoofer(drive, indexer, tilt, flywheels, objective, driverAutoShoot.or(operatorScoreOverride)));
        operatorPodium.onTrue(new AutoScoreShooterPodium(drive, indexer, tilt, flywheels, objective, driverAutoShoot.or(operatorScoreOverride)));
        operatorJamClear.or(driverJamClear).whileTrue(new IndexerJamClearing(arm, intake, indexer));
        // operatorIntakeGroundToHold.onTrue(new IntakeNoteGroundHold(arm, intake, objective));
        // operatorAmp.onTrue(new AutoScoreAmp(drive, arm, intake, objective, operatorOverrideScore.or(driverAutoShoot)));
        operatorAmp.onTrue(new AutoScoreShooterAmp(drive, indexer, tilt, flywheels, objective, driverAutoShoot.or(operatorScoreOverride)));

        operatorClimbRaise.onTrue(new ClimbAutoRaise(climb, objective));
        operatorClimbPull.onTrue(new ClimbPoweredRetract(climb, objective));
        operatorClimbReset.onTrue(new ClimbReset(climb, objective));
        operatorClimbManual.onTrue(new ClimbManualOverride(climb, objective, operatorClimbThrottle));

        operatorResetMotionPlanner.onTrue(new InstantCommand(() -> arm.setResetMotionPlanner(true), arm));
        operatorResetMotionPlanner.onFalse(new InstantCommand(() -> arm.setResetMotionPlanner(false), arm));

        // operatorOverrideScore.and(robotTeleopEnabled).whileTrue(ArmCommandFactory.alignStateOverrideButton(drive));

        // Operator override switches
        // armForceEnable
        // armHomingSequence
        // overrideArmSafety
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private ControllerDriveInputs getDriveInputs() {
        return new ControllerDriveInputs(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX()).applyDeadZone(0.03, 0.03, 0.03, 0.05).powerPolar(2);
    }

    public boolean hasConfigErrors() {
        return false;
    }
}
