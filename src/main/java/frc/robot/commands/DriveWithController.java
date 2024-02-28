// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.Utility;
import frc.robot.lib.dashboard.LoggedTunableBoolean;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.lib.drive.ControllerDriveInputs;
import frc.robot.lib.drive.SwerveHeadingController;
import frc.robot.lib.drive.AutoAlignPointSelector.RequestedAlignment;
import frc.robot.lib.drive.SwerveHeadingController.HeadingControllerState;
import frc.robot.lib.util.TimeDelayedBoolean;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

public class DriveWithController extends Command {
    private static final LoggedTunableBoolean mUseOpenLoopDrive = new LoggedTunableBoolean("/Drive/UseOpenLoop", Constants.kDriveUseOpenLoop);

    private final Drive drive;
    private final ObjectiveTracker objective;
    private final Supplier<ControllerDriveInputs> driveInputSupplier;
    private final Supplier<Boolean> slowModeSupplier;
    private final Supplier<Boolean> disableFieldOrient;
    private final Supplier<Boolean> snapAutoAlignAngle;
    private final Supplier<Boolean> snapAutoAlignIgnoringPreferred;

    private boolean mUseOpenLoop = false;

    private final TimeDelayedBoolean mShouldMaintainHeading = new TimeDelayedBoolean();
    private final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    private Optional<Double> mHeadingGoal = Optional.empty();

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
            ObjectiveTracker objective,
            Supplier<ControllerDriveInputs> driveInputSupplier,
            Supplier<Boolean> slowModeSupplier,
            Supplier<Boolean> disableFieldOrient,
            Supplier<Boolean> snapAutoAlign,
            Supplier<Boolean> snapAutoAlignIngoringPreferred
        ) {
        addRequirements(drive);

        this.drive = drive;
        this.objective = objective;
        this.driveInputSupplier = driveInputSupplier;
        this.slowModeSupplier = slowModeSupplier;
        this.disableFieldOrient = disableFieldOrient;
        this.snapAutoAlignAngle = snapAutoAlign;
        this.snapAutoAlignIgnoringPreferred = snapAutoAlignIngoringPreferred;
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
        if (mUseOpenLoopDrive.hasChanged(hashCode())) {
            mUseOpenLoop = mUseOpenLoopDrive.get();
        }

        var linearSpeedFactor = linearSpeedLimitChooser.getSelected();
        var angularSpeedFactor = angularSpeedLimitChooser.getSelected();
        if (slowModeSupplier.get()) {
            linearSpeedFactor *= 0.5;
            angularSpeedFactor *= 0.5;
        }

        var controllerInputs = driveInputSupplier.get()
        .times(linearSpeedFactor, angularSpeedFactor);

        boolean drive_turning = !Util.epsilonEquals(controllerInputs.getRotation(), 0);
        boolean drive_translating = Utility.getSpeedAsScalar(drive.getMeasuredSpeeds()) >= 0.1;

        boolean shouldSnapAutoAlignAngle = snapAutoAlignAngle.get();
        boolean shouldSnapAutoAlignIgnoringPreferred = snapAutoAlignIgnoringPreferred.get();
        boolean autoMaintain = mShouldMaintainHeading.update(!drive_turning && drive_translating && !shouldSnapAutoAlignAngle, 0.2);

        if (shouldSnapAutoAlignAngle) {
            mHeadingGoal = Optional.of(AutoAlignPointSelector.getAlignTarget(drive.getPose(), objective.getRequestedAlignment(false)).orElse(drive.getPose()).getRotation().getDegrees());
        } else if (shouldSnapAutoAlignIgnoringPreferred) {
            mHeadingGoal = Optional.of(AutoAlignPointSelector.getAlignTarget(drive.getPose(), objective.getRequestedAlignment(true)).orElse(drive.getPose()).getRotation().getDegrees());
        } else if (!autoMaintain) {
            mHeadingGoal = Optional.of(drive.getPose().getRotation().getDegrees());
        }

        if (autoMaintain || shouldSnapAutoAlignAngle) {
            mHeadingGoal.ifPresent(mSwerveHeadingController::setGoal);
            if (mSwerveHeadingController.getAbsError() <= Constants.kSwerveHeadingControllerMaintainThreshold) {
                mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.MAINTAIN);
            } else {
                mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.SNAP);
            }
        } else {
            mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
        }

        if (mSwerveHeadingController.getHeadingControllerState() != HeadingControllerState.OFF) {
            controllerInputs.setRotation(mSwerveHeadingController.update(drive.getPose().getRotation().getDegrees()));
        }

        var speeds = controllerInputs.getVelocityFieldOriented(
            Constants.kMaxVelocityMetersPerSecond, 
            Constants.kMaxAngularVelocityRadiansPerSecond, 
            (disableFieldOrient.get() ? new Rotation2d() : drive.getPose().getRotation()).plus(drive.getGyroYawVelocity().times(Constants.kDriveTeleopAngleBiasFactor))
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

    private double getClosestCardinal(double robotAngle) {
        return Math.round(robotAngle/180.0) * 180;
    }
}
