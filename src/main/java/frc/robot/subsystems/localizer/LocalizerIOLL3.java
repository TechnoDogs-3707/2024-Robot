package frc.robot.subsystems.localizer;

import static frc.robot.subsystems.localizer.LocalizerConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.Results;

public class LocalizerIOLL3 implements LocalizerIO {
    public static final double kNetworkTablesDelaySeonds = 0.005;
    public final DoubleTopic kLLLatencyTopic;
    public final DoubleSubscriber kLLLatencySub;

    // max allowable networktables delay; we use this to determine if LL3 is connected
    public static final double kNetworkTablesLatencyThreshold = 5.0;

    public LocalizerIOLL3() {
        kLLLatencyTopic = NetworkTableInstance.getDefault().getDoubleTopic("limelight/tl");
        kLLLatencySub = kLLLatencyTopic.subscribe(0);

        // TODO: Change limelight to use pinhole camera model
    }

    @Override
    public void updateInputs(LocalizerIOInputs inputs) {
        // use the assumption that recent NT data means that limelight is connected
        inputs.visionConnected = true; // TODO: make sure we can detect if we are connected or not

        if (inputs.visionConnected) {
            Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;

            inputs.position = results.botpose_wpiblue;
            inputs.stddevs = getStdDevs(results).getData();

            inputs.targetsVisible = results.targets_Fiducials.length;

            inputs.lastUpdateTimestamp = Timer.getFPGATimestamp();// - results.latency_capture/1000.0 - results.latency_pipeline/1000.0 - results.latency_jsonParse/1000.0;

            inputs.poseValid = results.valid;
        } else {
            inputs.poseValid = false;
        }
    }

    private Matrix<N3, N1> getStdDevs(Results results) {
        // rewritten for LL3 based on code used by photonvision
        var estStdDevs = kSingleTagStdDevs;
        var targets = results.targets_Fiducials;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            double tagDist = tgt.getTargetPose_CameraSpace().getTranslation().getNorm();
            numTags++;
            avgDist += tagDist;
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    // private double[] convertPoseArrayToRadians(double[] poseArray) {
    //     for (int i = 3; i < 6; i++) {
    //         poseArray[i] = Math.toRadians(poseArray[i]);
    //     }
    //     return poseArray;
    // }
}
