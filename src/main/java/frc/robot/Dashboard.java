package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveWithController;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.SupplierWidget;
import frc.robot.lib.dashboard.WidgetConfig;
import frc.robot.lib.dashboard.SendableWidget;
import frc.robot.lib.dashboard.Alert.SendableAlerts;
import frc.robot.subsystems.controllerFeedback.ControllerFeedback;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.Drive.DriveControlState;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.localizer.Localizer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.tilt.ShooterTilt;

public class Dashboard {
    // Drive Tab
    public final String driveTabName = "Drive";
    public final SendableWidget<SendableChooser<Double>> drive_linearSpeedChooser;
    public final SendableWidget<SendableChooser<Double>> drive_angularSpeedChooser;
    public final SupplierWidget<String> drive_activeDriveController;
    public final SupplierWidget<String> drive_activeKinematicLimits;
    public final SendableWidget<SendableChooser<Boolean>> drive_enableVision;
    
    public final SendableWidget<SendableChooser<Boolean>> drive_steerMotorBrake;
    
    public final SupplierWidget<Double> drive_mod0_position;
    public final SupplierWidget<Double> drive_mod1_position;
    public final SupplierWidget<Double> drive_mod2_position;
    public final SupplierWidget<Double> drive_mod3_position;
    
    // Setup Tab
    public final String setupTabName = "Setup";
    public final SendableWidget<SendableAlerts> setup_alerts;
    public final SendableWidget<SendableChooser<String>> setup_autonMode;

    public final SendableWidget<SendableChooser<String>> setup_preferredSourceSide;
    public final SendableWidget<SendableChooser<String>> setup_preferredSpeakerLocation;

    public final SendableWidget<Field2d> setup_initPreview;
    public final SendableWidget<SendableChooser<String>> setup_poseWidgetSource;
    public final SendableWidget<SendableChooser<String>> setup_poseInitSource;
    public final SendableWidget<SendableChooser<String>> setup_useAutoAlignInAuto;

    // TODO: link a shuffleboard widget to a smartdashboard number
    // public final SendableWidget<LoggedDashboardNumber> setup_endgameAlert1;
    // public final SendableWidget<SendableLoggedDashboardNumber> setup_endgameAlert2;
    
    // Match Tab
    public static final String matchTabName = "Match";
    
    public final SendableWidget<SendableAlerts> match_alerts;

    public final SupplierWidget<String> match_ledColorPreview;

    public final SupplierWidget<Double> match_boostTimer;
    public final SupplierWidget<Boolean> match_boostActive;
    public final SupplierWidget<Boolean> match_boostAvailable;
    
    public final SendableWidget<Command> match_homeIntake;
    // public final SendableWidget<Command> match_homeTilt;

    // Testing Tab
    public static final String testingTabName = "Testing";

    public final SendableWidget<SendableAlerts> testing_alerts;

    public final SendableWidget<SendableChooser<String>> testing_enableControllerFeedback;
    public final SendableWidget<SendableChooser<String>> testing_enableFullBrightnessLEDS;
    public final SendableWidget<SendableChooser<String>> testing_forceBrakeMode;

    // public final SendableWidget<SendableChooser<String>> testing_preloadSimGripper;
    
    public Dashboard(Robot robot, RobotContainer container, Drive drive, IntakeDeploy intakeDeploy, Intake intake, Flywheels flywheels, ShooterTilt tilt, Indexer indexer,  LED led, Localizer vision, ObjectiveTracker objective, ControllerFeedback controllerFeedback) {
        // Drive Tab
        drive_linearSpeedChooser = new SendableWidget<SendableChooser<Double>>(driveTabName, "Linear Speed Limit",
        DriveWithController.linearSpeedLimitChooser,
        new WidgetConfig(0, 0, 2, 1, BuiltInWidgets.kComboBoxChooser));
        drive_angularSpeedChooser = new SendableWidget<SendableChooser<Double>>(driveTabName, "Angular Speed Limit",
        DriveWithController.angularSpeedLimitChooser,
        new WidgetConfig(2, 0, 2, 1, BuiltInWidgets.kComboBoxChooser));
        drive_activeDriveController = new SupplierWidget<String>(driveTabName, "Drive Control State", "", () -> drive.getControlState().title,
        new WidgetConfig(0, 1, 2, 1, BuiltInWidgets.kTextView));
        drive_activeKinematicLimits = new SupplierWidget<String>(driveTabName, "Current Kinematic Limits", "", () -> drive.getKinematicLimitsTitle(),
        new WidgetConfig(2, 1, 2, 1, BuiltInWidgets.kTextView));
        drive_enableVision = new SendableWidget<SendableChooser<Boolean>>(driveTabName, "Use AprilTags",
        vision.mVisionEnableChooser, new WidgetConfig(0, 2, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        
        drive_steerMotorBrake = new SendableWidget<SendableChooser<Boolean>>(driveTabName, "Steer Motor Neutral Mode",
        SwerveModule.steerNeutralMode, new WidgetConfig(4, 2, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        
        drive_mod0_position = new SupplierWidget<Double>(driveTabName, "Position 0", 0.0, () -> drive.getLastEncoderPosition(0), new WidgetConfig(6, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod1_position = new SupplierWidget<Double>(driveTabName, "Position 1", 0.0, () -> drive.getLastEncoderPosition(1), new WidgetConfig(7, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod2_position = new SupplierWidget<Double>(driveTabName, "Position 2", 0.0, () -> drive.getLastEncoderPosition(2), new WidgetConfig(8, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod3_position = new SupplierWidget<Double>(driveTabName, "Position 3", 0.0, () -> drive.getLastEncoderPosition(3), new WidgetConfig(9, 2, 1, 1, BuiltInWidgets.kTextView));
        
        // Setup Tab
        setup_alerts = new SendableWidget<Alert.SendableAlerts>(setupTabName, "Master Alerts", Alert.getDefaultAlertGroup(), new WidgetConfig(6, 0, 4, 4, "Alerts"));
        
        setup_autonMode = new SendableWidget<SendableChooser<String>>(setupTabName, "Autonomous Mode", container.autoChooser.getSendableChooser(), new WidgetConfig(0, 0, 4, 1, BuiltInWidgets.kComboBoxChooser));
        
        setup_preferredSourceSide = new SendableWidget<SendableChooser<String>>(setupTabName, "Preferred Source", objective.mAutoAlignSourcePreference.getSendableChooser(), new WidgetConfig(4, 1, 2, 1, BuiltInWidgets.kComboBoxChooser));
        setup_preferredSpeakerLocation = new SendableWidget<SendableChooser<String>>(setupTabName, "Preferred Speaker", objective.mAutoAlignSpeakerPreference.getSendableChooser(), new WidgetConfig(4, 2, 2, 1, BuiltInWidgets.kComboBoxChooser));

        setup_initPreview = new SendableWidget<Field2d>(setupTabName, "Initial Pose Preview", drive.mPosePreviewSource, new WidgetConfig(0, 1, 4, 2, BuiltInWidgets.kField));
        setup_poseWidgetSource = new SendableWidget<SendableChooser<String>>(setupTabName, "Pose Widget Source", drive.mPoseWidgetUsePreview.getChooser(), new WidgetConfig(0, 3, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        setup_poseInitSource = new SendableWidget<SendableChooser<String>>(setupTabName, "Pose Init Source", drive.mPoseInitFromEstimator.getChooser(), new WidgetConfig(2, 3, 2, 1, BuiltInWidgets.kSplitButtonChooser));

        setup_useAutoAlignInAuto = new SendableWidget<SendableChooser<String>>(setupTabName, "Use AutoAlign in Auto", drive.mUseAutoAlignDuringAuto.getChooser(), new WidgetConfig(4, 3, 2, 1, BuiltInWidgets.kSplitButtonChooser));

        // Match Tab
        match_alerts = new SendableWidget<Alert.SendableAlerts>(matchTabName, "Alerts", Alert.getDefaultAlertGroup(), new WidgetConfig(0, 0, 4, 4, "Alerts"));
        
        match_ledColorPreview = new SupplierWidget<String>(matchTabName, "LED Preview", "#000000", led::getPreviewColor, new WidgetConfig(4, 0, 3, 1, "Single Color View"));

        match_boostTimer = new SupplierWidget<Double>(matchTabName, "Shooting Offset", 0.0, () -> RobotState.getInstance().shooterTweakRotations, new WidgetConfig(7, 0, 3, 1, BuiltInWidgets.kTextView));
        match_boostActive = new SupplierWidget<Boolean>(matchTabName, "Boost Active", false, () -> false, new WidgetConfig(6, 1, 2, 1, BuiltInWidgets.kBooleanBox));
        match_boostAvailable = new SupplierWidget<Boolean>(matchTabName, "Boost Available", false, () -> false, new WidgetConfig(8, 1, 2, 1, BuiltInWidgets.kBooleanBox));

        match_homeIntake = new SendableWidget<Command>(matchTabName, "Home Intake", intakeDeploy.homeEncoderCommand(), new WidgetConfig(6, 2, 2, 1, BuiltInWidgets.kCommand));

        //Testing Tab
        testing_alerts = new SendableWidget<Alert.SendableAlerts>(testingTabName, "Alerts", Alert.getDefaultAlertGroup(), new WidgetConfig(6, 0, 4, 4, "Alerts"));

        testing_enableControllerFeedback = new SendableWidget<SendableChooser<String>>(testingTabName, "Controller Feedback off FMS", controllerFeedback.mEnableVibrationOffFMS.getChooser(), new WidgetConfig(0, 0, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        testing_enableFullBrightnessLEDS = new SendableWidget<SendableChooser<String>>(testingTabName, "LED Brightness off FMS", led.dashboardLEDBrightnessOverride.getChooser(), new WidgetConfig(0, 1, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        testing_forceBrakeMode = new SendableWidget<SendableChooser<String>>(testingTabName, "Off-FMS Brake Mode Setting", drive.mForceBrakeModeSwitch.getChooser(), new WidgetConfig(0, 2, 2, 1, BuiltInWidgets.kSplitButtonChooser));
    
        // testing_preloadSimGripper = new SendableWidget<SendableChooser<String>>(testingTabName, "SimGripper Preload Mode", GripperIOSim.mPreloadGripperInAuton.getChooser(), new WidgetConfig(2, 1, 2, 1, BuiltInWidgets.kSplitButtonChooser));
    }
    
    public void resetWidgets() {
        //TODO: Reset widgets
    }
}
