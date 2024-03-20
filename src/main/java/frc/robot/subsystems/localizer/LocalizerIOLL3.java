package frc.robot.subsystems.localizer;

import static frc.robot.subsystems.localizer.LocalizerConstants.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
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
        Optional<LimelightResults> results = LimelightHelpers.getLatestResults("limelight");

        if (results.isPresent()) {
            var tgtResults = results.get().targetingResults;

            inputs.position = tgtResults.botpose_wpiblue;
            // inputs.stddevs = getStdDevs(results).getData();
            inputs.stddevs = getStdDevsTrusting(tgtResults).getData();

            inputs.targetsVisible = tgtResults.targets_Fiducials.length;

            inputs.lastUpdateTimestamp = (Logger.getRealTimestamp()/1_000_000.0) - (tgtResults.latency_capture/1000.0) - (tgtResults.latency_pipeline/1000.0) - (tgtResults.latency_jsonParse/1000.0);

            inputs.poseValid = tgtResults.valid;
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

    private Matrix<N3, N1> getStdDevsTrusting(Results results) {
        return kMultiTagStdDevs;
    }

    // private double[] convertPoseArrayToRadians(double[] poseArray) {
    //     for (int i = 3; i < 6; i++) {
    //         poseArray[i] = Math.toRadians(poseArray[i]);
    //     }
    //     return poseArray;
    // }
}
