// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.localizer;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotState.VisionObservation;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.util.poofsUtils.VirtualSubsystem;

/** Add your docs here. */
public class Localizer extends VirtualSubsystem {
    private final LocalizerIO mIO;
    private final LocalizerIOInputsAutoLogged mInputs;

    private final Consumer<VisionObservation> mConsumer;

    public final SendableChooser<Boolean> mVisionEnableChooser = new SendableChooser<>();

    private boolean mEnableVisionUpdates = true;
    private Alert mAlertVisionUpdatesDisabled =
        new Alert("Vision updates are manually disabled.", AlertType.WARNING);

    private Alert mAlertVisionNotConnected = 
        new Alert("Vision Source Disconnected!", AlertType.WARNING);

    public Localizer(LocalizerIO io, Consumer<VisionObservation> consumer) {
        this.mIO = io;
        this.mInputs = new LocalizerIOInputsAutoLogged();
        this.mConsumer = consumer;

        mVisionEnableChooser.setDefaultOption("Enabled", true);
        mVisionEnableChooser.addOption("Disabled", false);
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Localizer", mInputs);

        mAlertVisionUpdatesDisabled.set(!mEnableVisionUpdates);
        mAlertVisionNotConnected.set(!mInputs.visionConnected);

        Logger.recordOutput("Localizer/DashboardEnabledVisionUpdates", mEnableVisionUpdates);

        if (mInputs.visionConnected && mInputs.poseValid && mEnableVisionUpdates) {
            Pose3d pose = new Pose3d(
                new Translation3d(
                    mInputs.position[0], 
                    mInputs.position[1], 
                    mInputs.position[2]
                    ), 
                new Rotation3d(
                    mInputs.position[3], 
                    mInputs.position[4], 
                    mInputs.position[5]
                    )
            );
            Matrix<N3, N1> stddevs = VecBuilder.fill(
                mInputs.stddevs[0], 
                mInputs.stddevs[1], 
                Double.MAX_VALUE// mInputs.stddevs[2]
            );

            Logger.recordOutput("Localizer/LastPose2d", pose.toPose2d());
            Logger.recordOutput("Localizer/LastStdDevs", stddevs.getData());
            Logger.recordOutput("Localizer/LastTimestamp", mInputs.lastUpdateTimestamp);
            mConsumer.accept(new VisionObservation(pose.toPose2d(), mInputs.lastUpdateTimestamp, stddevs));
        }
    }

    public void setVisionUpdatesEnabled(boolean enabled) {
        mEnableVisionUpdates = enabled;
    }
}
