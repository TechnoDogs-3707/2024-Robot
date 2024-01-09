package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveWithController;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.SendableTriggerButton;
import frc.robot.lib.dashboard.SupplierWidget;
import frc.robot.lib.dashboard.WidgetConfig;
import frc.robot.lib.dashboard.SendableWidget;
import frc.robot.lib.dashboard.Alert.SendableAlerts;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.GripperIOSim;
import frc.robot.subsystems.arm.Arm.GameObjectType;
import frc.robot.subsystems.controllerFeedback.ControllerFeedback;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.Drive.DriveControlState;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.localizer.Localizer;

public class Dashboard {
    // Drive Tab
    public final String driveTabName = "Drive";
    public final SendableWidget<SendableChooser<Double>> drive_linearSpeedChooser;
    public final SendableWidget<SendableChooser<Double>> drive_angularSpeedChooser;
    public final SupplierWidget<String> drive_activeDriveController;
    public final SupplierWidget<String> drive_activeKinematicLimits;
    public final SendableWidget<SendableChooser<Boolean>> drive_enableVision;
    
    public final SendableWidget<SendableChooser<Boolean>> drive_allowEncoderCal;
    public final SendableWidget<SendableTriggerButton> drive_saveEncoderOffsets;
    public final SendableWidget<SendableTriggerButton> drive_restoreEncoderOffsets;
    public final SendableWidget<SendableChooser<Boolean>> drive_steerMotorBrake;
    public final SendableWidget<SendableTriggerButton> drive_refreshOffsets;
    
    public final SendableWidget<SendableTriggerButton> drive_mod0Cal;
    public final SendableWidget<SendableTriggerButton> drive_mod1Cal;
    public final SendableWidget<SendableTriggerButton> drive_mod2Cal;
    public final SendableWidget<SendableTriggerButton> drive_mod3Cal;
    
    public final SupplierWidget<Double> drive_mod0_offset;
    public final SupplierWidget<Double> drive_mod1_offset;
    public final SupplierWidget<Double> drive_mod2_offset;
    public final SupplierWidget<Double> drive_mod3_offset;
    
    public final SupplierWidget<Double> drive_mod0_position;
    public final SupplierWidget<Double> drive_mod1_position;
    public final SupplierWidget<Double> drive_mod2_position;
    public final SupplierWidget<Double> drive_mod3_position;
    
    // Setup Tab
    public final String setupTabName = "Setup";
    public final SendableWidget<SendableAlerts> setup_alerts;
    public final SupplierWidget<Boolean> setup_redAlliance;
    public final SupplierWidget<Boolean> setup_blueAlliance;
    public final SendableWidget<SendableChooser<String>> setup_autonMode;

    public final SendableWidget<Field2d> setup_initPreview;
    public final SendableWidget<SendableChooser<String>> setup_poseWidgetSource;
    public final SendableWidget<SendableChooser<String>> setup_poseInitSource;
    public final SendableWidget<SendableChooser<String>> setup_useAprilTagsInAuto;

    // TODO: link a shuffleboard widget to a smartdashboard number
    // public final SendableWidget<LoggedDashboardNumber> setup_endgameAlert1;
    // public final SendableWidget<SendableLoggedDashboardNumber> setup_endgameAlert2;
    
    // Match Tab
    public static final String matchTabName = "Match";
    
    public final SendableWidget<SendableAlerts> match_alerts;
    public final SupplierWidget<Double> match_batterySoC;
    public final SupplierWidget<Boolean> match_isFieldOriented;
    
    public final SupplierWidget<Double> match_FLTemp;
    public final SupplierWidget<Double> match_FRTemp;
    public final SupplierWidget<Double> match_RLTemp;
    public final SupplierWidget<Double> match_RRTemp;
    
    public final SupplierWidget<Boolean> match_autoAlignFinished;
    public final SupplierWidget<Boolean> match_autoAlignStarted;
    public final SupplierWidget<Boolean> match_coneMode;
    public final SupplierWidget<Boolean> match_cubeMode;
    
    public final SupplierWidget<Boolean> match_scoreHigh;
    public final SupplierWidget<Boolean> match_scoreMid;
    public final SupplierWidget<Boolean> match_scoreLow;
    public final SupplierWidget<Boolean> match_stowTransport;

    // Testing Tab
    public static final String testingTabName = "Testing";

    public final SendableWidget<SendableAlerts> testing_alerts;

    public final SendableWidget<SendableChooser<String>> testing_enableControllerFeedback;
    public final SendableWidget<SendableChooser<String>> testing_enableFullBrightnessLEDS;
    public final SendableWidget<SendableChooser<String>> testing_forceBrakeMode;

    public final SendableWidget<SendableChooser<String>> testing_preloadSimGripper;
    
    public Dashboard(Robot robot, RobotContainer container, Drive drive, Arm arm, LED led, Localizer vision, ControllerFeedback controllerFeedback) {
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
        
        drive_allowEncoderCal = new SendableWidget<SendableChooser<Boolean>>(driveTabName, "Encoder Updates",
        drive.mEncoderUpdateChooser, new WidgetConfig(4, 0, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        
        drive_saveEncoderOffsets = new SendableWidget<SendableTriggerButton>(driveTabName, "Save Offsets",
        new SendableTriggerButton("Save", () -> {
            drive.saveEncoderOffsetsToPersist();
        }), new WidgetConfig(4, 1, 1, 1, BuiltInWidgets.kCommand));
        
        drive_restoreEncoderOffsets = new SendableWidget<SendableTriggerButton>(driveTabName, "Restore Offsets",
        new SendableTriggerButton("Restore", () -> {
            drive.updateEncoderOffsetsFromPersist();
        }), new WidgetConfig(5, 1, 1, 1, BuiltInWidgets.kCommand));
        drive_steerMotorBrake = new SendableWidget<SendableChooser<Boolean>>(driveTabName, "Steer Motor Neutral Mode",
        SwerveModule.steerNeutralMode, new WidgetConfig(4, 2, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        
        drive_refreshOffsets = new SendableWidget<SendableTriggerButton>(driveTabName, "Offset Cache",
        new SendableTriggerButton("Refresh", () -> drive.refreshEncoderOffsets()),
        new WidgetConfig(9, 3, 1, 1, BuiltInWidgets.kCommand));
        
        drive_mod0Cal = new SendableWidget<SendableTriggerButton>(driveTabName, "Moddule 0",
        new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(0)),
        new WidgetConfig(6, 0, 1, 1, BuiltInWidgets.kCommand));
        drive_mod1Cal = new SendableWidget<SendableTriggerButton>(driveTabName, "Moddule 1",
        new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(1)),
        new WidgetConfig(7, 0, 1, 1, BuiltInWidgets.kCommand));
        drive_mod2Cal = new SendableWidget<SendableTriggerButton>(driveTabName, "Moddule 2",
        new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(2)),
        new WidgetConfig(8, 0, 1, 1, BuiltInWidgets.kCommand));
        drive_mod3Cal = new SendableWidget<SendableTriggerButton>(driveTabName, "Moddule 3",
        new SendableTriggerButton("Calibrate", () -> drive.zeroEncoder(3)),
        new WidgetConfig(9, 0, 1, 1, BuiltInWidgets.kCommand));
        
        drive_mod0_offset = new SupplierWidget<Double>(driveTabName, "Offset 0", 0.0, () -> drive.getCachedEncoderOffsets(0), new WidgetConfig(6, 1, 1, 1, BuiltInWidgets.kTextView));
        drive_mod1_offset = new SupplierWidget<Double>(driveTabName, "Offset 1", 0.0, () -> drive.getCachedEncoderOffsets(1), new WidgetConfig(7, 1, 1, 1, BuiltInWidgets.kTextView));
        drive_mod2_offset = new SupplierWidget<Double>(driveTabName, "Offset 2", 0.0, () -> drive.getCachedEncoderOffsets(2), new WidgetConfig(8, 1, 1, 1, BuiltInWidgets.kTextView));
        drive_mod3_offset = new SupplierWidget<Double>(driveTabName, "Offset 3", 0.0, () -> drive.getCachedEncoderOffsets(3), new WidgetConfig(9, 1, 1, 1, BuiltInWidgets.kTextView));
        
        drive_mod0_position = new SupplierWidget<Double>(driveTabName, "Position 0", 0.0, () -> drive.getLastEncoderPosition(0), new WidgetConfig(6, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod1_position = new SupplierWidget<Double>(driveTabName, "Position 1", 0.0, () -> drive.getLastEncoderPosition(1), new WidgetConfig(7, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod2_position = new SupplierWidget<Double>(driveTabName, "Position 2", 0.0, () -> drive.getLastEncoderPosition(2), new WidgetConfig(8, 2, 1, 1, BuiltInWidgets.kTextView));
        drive_mod3_position = new SupplierWidget<Double>(driveTabName, "Position 3", 0.0, () -> drive.getLastEncoderPosition(3), new WidgetConfig(9, 2, 1, 1, BuiltInWidgets.kTextView));
        
        // Setup Tab
        setup_alerts = new SendableWidget<Alert.SendableAlerts>(setupTabName, "Master Alerts", Alert.getDefaultAlertGroup(), new WidgetConfig(6, 0, 4, 4, "Alerts"));
        
        setup_redAlliance = new SupplierWidget<Boolean>(setupTabName, "Red Alliance", false, () -> false, new WidgetConfig(0, 0, 1, 1, BuiltInWidgets.kBooleanBox));
        setup_blueAlliance = new SupplierWidget<Boolean>(setupTabName, "Blue Alliance", false, () -> false, new WidgetConfig(1, 0, 1, 1, BuiltInWidgets.kBooleanBox));
        setup_autonMode = new SendableWidget<SendableChooser<String>>(setupTabName, "Autonomous Mode", container.autoChooser.getSendableChooser(), new WidgetConfig(2, 0, 4, 1, BuiltInWidgets.kComboBoxChooser));
        
        setup_initPreview = new SendableWidget<Field2d>(setupTabName, "Initial Pose Preview", drive.mPosePreviewSource, new WidgetConfig(0, 1, 4, 2, BuiltInWidgets.kField));
        setup_poseWidgetSource = new SendableWidget<SendableChooser<String>>(setupTabName, "Pose Widget Source", drive.mPoseWidgetUsePreview.getChooser(), new WidgetConfig(0, 3, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        setup_poseInitSource = new SendableWidget<SendableChooser<String>>(setupTabName, "Pose Init Source", drive.mPoseInitFromTags.getChooser(), new WidgetConfig(2, 3, 2, 1, BuiltInWidgets.kSplitButtonChooser));

        setup_useAprilTagsInAuto = new SendableWidget<SendableChooser<String>>(setupTabName, "Use Fiducials in Auto", drive.mUseTagsDuringAuto.getChooser(), new WidgetConfig(4, 3, 2, 1, BuiltInWidgets.kSplitButtonChooser));

        // Match Tab
        match_alerts = new SendableWidget<Alert.SendableAlerts>(matchTabName, "Alerts", Alert.getDefaultAlertGroup(), new WidgetConfig(0, 0, 4, 4, "Alerts"));
        match_batterySoC = new SupplierWidget<Double>(matchTabName, "SoC", 0.0, () -> 0.0, new WidgetConfig(4, 0, 1, 3, BuiltInWidgets.kVoltageView));
        match_isFieldOriented = new SupplierWidget<Boolean>(matchTabName, "FieldOriented", false, () -> false, new WidgetConfig(4, 3, 1, 1, BuiltInWidgets.kBooleanBox));
        
        match_FLTemp = new SupplierWidget<Double>(matchTabName, "FL Temp", 0.0, () -> 0.0, new WidgetConfig(5, 0, 1, 1, BuiltInWidgets.kTextView));
        match_FRTemp = new SupplierWidget<Double>(matchTabName, "FR Temp", 0.0, () -> 0.0, new WidgetConfig(5, 1, 1, 1, BuiltInWidgets.kTextView));
        match_RLTemp = new SupplierWidget<Double>(matchTabName, "RL Temp", 0.0, () -> 0.0, new WidgetConfig(5, 2, 1, 1, BuiltInWidgets.kTextView));
        match_RRTemp = new SupplierWidget<Double>(matchTabName, "RR Temp", 0.0, () -> 0.0, new WidgetConfig(5, 3, 1, 1, BuiltInWidgets.kTextView));
        
        Supplier<Boolean> driveIsAligning = () -> {
            return drive.getControlState() == DriveControlState.AUTO_ALIGN || drive.getControlState() == DriveControlState.AUTO_ALIGN_Y_THETA;
        };
        
        Supplier<Boolean> isConeMode = () -> {
            return arm.getGameObject() == GameObjectType.CONE;
        };
        
        Supplier<Boolean> isCubeMode = () -> {
            return arm.getGameObject() == GameObjectType.CUBE;
        };
        
        match_autoAlignFinished = new SupplierWidget<Boolean>(matchTabName, "Align Finished", false, drive::autoAlignAtTarget, new WidgetConfig(6, 0, 2, 1, BuiltInWidgets.kBooleanBox));
        match_autoAlignStarted = new SupplierWidget<Boolean>(matchTabName, "Align Started", false, driveIsAligning, new WidgetConfig(6, 1, 2, 1, BuiltInWidgets.kBooleanBox));
        match_coneMode = new SupplierWidget<Boolean>(matchTabName, "Cone Mode", false, isConeMode, new WidgetConfig(6, 2, 2, 1, BuiltInWidgets.kBooleanBox));
        match_cubeMode = new SupplierWidget<Boolean>(matchTabName, "Cube Mode", false, isCubeMode, new WidgetConfig(6, 3, 2, 1, BuiltInWidgets.kBooleanBox));
        
        match_scoreHigh = new SupplierWidget<Boolean>(matchTabName, "Score High", false, () -> false, new WidgetConfig(8, 0, 2, 1, BuiltInWidgets.kBooleanBox));
        match_scoreMid = new SupplierWidget<Boolean>(matchTabName, "Score Mid", false, () -> false, new WidgetConfig(8, 1, 2, 1, BuiltInWidgets.kBooleanBox));
        match_scoreLow = new SupplierWidget<Boolean>(matchTabName, "Score Low", false, () -> false, new WidgetConfig(8, 2, 2, 1, BuiltInWidgets.kBooleanBox));
        match_stowTransport = new SupplierWidget<Boolean>(matchTabName, "Stow or Transport", false, () -> false, new WidgetConfig(8, 3, 2, 1, BuiltInWidgets.kBooleanBox));

        //Testing Tab
        testing_alerts = new SendableWidget<Alert.SendableAlerts>(testingTabName, "Alerts", Alert.getDefaultAlertGroup(), new WidgetConfig(6, 0, 4, 4, "Alerts"));

        testing_enableControllerFeedback = new SendableWidget<SendableChooser<String>>(testingTabName, "Controller Feedback off FMS", controllerFeedback.mEnableVibrationOffFMS.getChooser(), new WidgetConfig(0, 0, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        testing_enableFullBrightnessLEDS = new SendableWidget<SendableChooser<String>>(testingTabName, "LED Brightness off FMS", led.dashboardLEDBrightnessOverride.getChooser(), new WidgetConfig(0, 1, 2, 1, BuiltInWidgets.kSplitButtonChooser));
        testing_forceBrakeMode = new SendableWidget<SendableChooser<String>>(testingTabName, "Off-FMS Brake Mode Setting", drive.mForceBrakeModeSwitch.getChooser(), new WidgetConfig(0, 2, 2, 1, BuiltInWidgets.kSplitButtonChooser));
    
        testing_preloadSimGripper = new SendableWidget<SendableChooser<String>>(testingTabName, "SimGripper Preload Mode", GripperIOSim.mPreloadGripperInAuton.getChooser(), new WidgetConfig(2, 1, 2, 1, BuiltInWidgets.kSplitButtonChooser));
    }
    
    public void resetWidgets() {
        //TODO: Reset widgets
    }
}
