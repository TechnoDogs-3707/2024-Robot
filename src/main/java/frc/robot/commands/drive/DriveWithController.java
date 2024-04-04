// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.lib.Utility;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.lib.drive.SwerveHeadingController;
import frc.robot.lib.drive.SwerveHeadingController.HeadingControllerState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.poofsUtils.PoofsUtil;
import frc.robot.util.poofsUtils.TimeDelayedBoolean;

public class DriveWithController extends Command {
    private static final LoggedTunableBoolean mUseOpenLoopDrive = new LoggedTunableBoolean("/Drive/UseOpenLoop", Constants.kDriveUseOpenLoop);

    // these are set for blue alliance, since it is the default. We only flip if the DS reports red.
    private final Rotation2d kAmpAlignAngle = Rotation2d.fromRotations(-0.25);

    private final Drive drive;
    private final Supplier<ControllerDriveInputs> driveInputSupplier;
    private final BooleanSupplier slowModeSupplier;
    private final BooleanSupplier disableFieldOrient;
    private final BooleanSupplier snapPodium;
    private final BooleanSupplier snapAmp;

    private boolean mUseOpenLoop = false;

    private final TimeDelayedBoolean mShouldMaintainHeading = new TimeDelayedBoolean();
    private final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    private Optional<Double> mHeadingGoal = Optional.empty();

    //TODO: change these to logged dashboard choosers
    public static final SendableChooser<Double> linearSpeedLimitChooser = new SendableChooser<>();

    public static final SendableChooser<Double> angularSpeedLimitChooser = new SendableChooser<>();

    static {
        linearSpeedLimitChooser.setDefaultOption("--Competition Mode--", 1.0);
        linearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        linearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        linearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
        angularSpeedLimitChooser.setDefaultOption("--Competition Mode--", 1.0);
        angularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        angularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        angularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    }

    /** Creates a new DefaultDriveCommand. */
    public DriveWithController(
            Drive drive,
            Supplier<ControllerDriveInputs> driveInputSupplier,
            BooleanSupplier slowModeSupplier,
            BooleanSupplier disableFieldOrient,
            BooleanSupplier snapPodium,
            BooleanSupplier snapAmp
        ) {
        addRequirements(drive);

        this.drive = drive;
        this.driveInputSupplier = driveInputSupplier;
        this.slowModeSupplier = slowModeSupplier;
        this.disableFieldOrient = disableFieldOrient;
        this.snapPodium = snapPodium;
        this.snapAmp = snapAmp;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mShouldMaintainHeading.update(false, 0);
        mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mUseOpenLoopDrive.ifChanged(hashCode(), (v) -> mUseOpenLoop = v);
        boolean flipAngles = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

        var linearSpeedFactor = linearSpeedLimitChooser.getSelected();
        var angularSpeedFactor = angularSpeedLimitChooser.getSelected();
        if (slowModeSupplier.getAsBoolean()) {
            linearSpeedFactor *= 0.5;
            angularSpeedFactor *= 0.5;
        }

        var controllerInputs = driveInputSupplier.get()
        .times(linearSpeedFactor, angularSpeedFactor);

        boolean drive_turning = !PoofsUtil.epsilonEquals(controllerInputs.getRotation(), 0);
        boolean drive_translating = Utility.getSpeedAsScalar(drive.getMeasuredSpeeds()) >= 0.1;

        boolean shouldSnapPodium = snapPodium.getAsBoolean();
        boolean shouldSnapAmp = snapAmp.getAsBoolean();
        boolean autoMaintain = mShouldMaintainHeading.update(!drive_turning && drive_translating && !shouldSnapPodium && !shouldSnapAmp, 0.2);

        if (shouldSnapPodium) {
            var aimParams = RobotState.getInstance().getAimingParameters();
            if (aimParams.effectiveDistance() >= 5.5) {
                mHeadingGoal = Optional.of(RobotState.getInstance().getTargetAngleForMoonshot().getDegrees());
            } else {
                mHeadingGoal = Optional.of(aimParams.driveHeading().getDegrees());
            }
        } else if (shouldSnapAmp) {
            mHeadingGoal = Optional.of(kAmpAlignAngle.getDegrees());
        } else if (!autoMaintain) {
            mHeadingGoal = Optional.of(RobotState.getInstance().getEstimatedPose().getRotation().getDegrees());
        }

        if (autoMaintain || shouldSnapPodium || shouldSnapAmp) {
            mHeadingGoal.ifPresent(mSwerveHeadingController::setGoal);
            if (mSwerveHeadingController.getAbsError() <= Constants.kSwerveHeadingControllerMaintainThreshold) {
                mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.MAINTAIN);
            } else {
                mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.SNAP);
            }
        } else {
            mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
        }

        Rotation2d angleWithFlip = flipAngles ? currentPose.getRotation().plus(Rotation2d.fromDegrees(180)) : currentPose.getRotation();

        // Rotation2d angleWithTempDisable = disableFieldOrient.getAsBoolean() ? (flipAngles ? Rotation2d.fromDegrees(180) : new Rotation2d()) : currentPose.getRotation();

        // Rotation2d angleWithFlipOnly = flipAngles ? currentPose.getRotation().plus(Rotation2d.fromDegrees(180)) : currentPose.getRotation();

        // Rotation2d angleWithFlipAndDisable = flipAngles ? angleWithTempDisable.plus(Rotation2d.fromDegrees(180)) : angleWithTempDisable;

        Rotation2d angleWithFlipAndDisable = disableFieldOrient.getAsBoolean() ? new Rotation2d() : angleWithFlip;

        if (mSwerveHeadingController.getHeadingControllerState() != HeadingControllerState.OFF) {
            controllerInputs.setRotation(mSwerveHeadingController.update(currentPose.getRotation().getDegrees()));
        }

        var speeds = controllerInputs.getVelocityFieldOriented(
            Constants.kMaxVelocityMetersPerSecond, 
            Constants.kMaxAngularVelocityRadiansPerSecond, 
            angleWithFlipAndDisable.plus(drive.getGyroYawVelocity().times(Constants.kDriveTeleopAngleBiasFactor))
        );
        
        if (mUseOpenLoop) {
            drive.setVelocityOpenLoop(speeds);
        } else {
            drive.setVelocityClosedLoop(speeds);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
        mShouldMaintainHeading.update(false, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // private double getClosestCardinal(double robotAngle) {
    //     return Math.round(robotAngle/180.0) * 180;
    // }
}
