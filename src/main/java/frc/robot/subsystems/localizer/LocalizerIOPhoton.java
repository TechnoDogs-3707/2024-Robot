// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.localizer;

// import static frc.robot.Constants.VisionSubsystem.*;

// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;

// /** Add your docs here. */
// public class LocalizerIOPhoton implements LocalizerIO {
//     private final PhotonCamera camera;
//     private final PhotonPoseEstimator poseEstimator;
//     private double lastEstTimestamp = 0.0;

//     public LocalizerIOPhoton() {
//         camera = new PhotonCamera(kCameraName);

//         poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP, camera, kRobotToCam);
//         poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     @Override
//     public void updateInputs(LocalizerIOInputs inputs) {
//         inputs.visionConnected = camera.isConnected();

//         Optional<EstimatedRobotPose> est = getEstimatedGlobalPose();
//         if (est.isPresent()) {
//             var pose = est.get();
//             inputs.poseValid = true;
//             inputs.position = pose3dToDoubleArray(pose.estimatedPose);
//             inputs.stddevs = getEstimationStdDevs(pose.estimatedPose.toPose2d()).getData();
//             inputs.lastUpdateTimestamp = pose.timestampSeconds;
//             inputs.targetsVisible = pose.targetsUsed.size();
//         } else {
//             inputs.poseValid = false;
//         }
//     }

//     private PhotonPipelineResult getLatestResult() {
//         return camera.getLatestResult();
//     }

//     /**
//      * The latest estimated robot pose on the field from vision data. This may be empty. This should
//      * only be called once per loop.
//      *
//      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
//      *     used for estimation.
//      */
//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
//         var visionEst = poseEstimator.update();
//         double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
//         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
//         if (newResult) lastEstTimestamp = latestTimestamp;
//         return visionEst;
//     }

//     /**
//      * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
//      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
//      * This should only be used when there are targets visible.
//      *
//      * @param estimatedPose The estimated pose to guess standard deviations for.
//      */
//     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
//         var estStdDevs = kSingleTagStdDevs;
//         var targets = getLatestResult().getTargets();
//         int numTags = 0;
//         double avgDist = 0;
//         for (var tgt : targets) {
//             var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
//             if (tagPose.isEmpty()) continue;
//             numTags++;
//             avgDist +=
//                     tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
//         }
//         if (numTags == 0) return estStdDevs;
//         avgDist /= numTags;
//         // Decrease std devs if multiple targets are visible
//         if (numTags > 1) estStdDevs = kMultiTagStdDevs;
//         // Increase std devs based on (average) distance
//         if (numTags == 1 && avgDist > 4)
//             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//         else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

//         return estStdDevs;
//     }

//     private double[] pose3dToDoubleArray(Pose3d pose) {
//         double[] array = new double[6];
//         array[0] = pose.getX();
//         array[1] = pose.getY();
//         array[2] = pose.getZ();
//         array[3] = pose.getRotation().getX();
//         array[4] = pose.getRotation().getY();
//         array[5] = pose.getRotation().getZ();

//         return array;
//     }
// }
