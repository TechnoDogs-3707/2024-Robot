// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.NoSuchElementException;
import lombok.Getter;
import lombok.Setter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    public record OdometryObservation(
    SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}
    
    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
    
    public record AimingParameters(
    Rotation2d driveHeading,
    Rotation2d tiltAngle,
    double effectiveDistance,
    double driveFeedVelocity,
    double degreesOffAxis) {}
    
    private static final LoggedTunableNumber lookahead =
    new LoggedTunableNumber("RobotState/lookaheadS", 0.0);
    private static final double poseBufferSizeSeconds = 2.0;
    
    /** Shooter angle look up table key: meters, values: rotations */
    private static final InterpolatingDoubleTreeMap tiltAngleMap = new InterpolatingDoubleTreeMap();
        
    static {
        tiltAngleMap.put(4.74, 0.004);
        tiltAngleMap.put(4.51, 0.01);
        tiltAngleMap.put(4.25, 0.012);
        tiltAngleMap.put(4.02, 0.013);
        tiltAngleMap.put(3.75, 0.016);
        tiltAngleMap.put(3.5, 0.02);
        tiltAngleMap.put(3.25, 0.022);
        tiltAngleMap.put(3.0, 0.0266);
        tiltAngleMap.put(2.75, 0.0307);
        tiltAngleMap.put(2.49, 0.036);
        tiltAngleMap.put(2.25, 0.041);
        tiltAngleMap.put(2.0, 0.049);
        tiltAngleMap.put(1.73, 0.056);
        tiltAngleMap.put(1.51, 0.062);
        tiltAngleMap.put(1.248, 0.07);
        tiltAngleMap.put(1.0, 0.07);
    }

    /** Table to compensate for shooting off axis
     * ; Key: degrees off axis (unsigned), Value: shooter rotations to add */
    private static final InterpolatingDoubleTreeMap offAxisCompMap = new InterpolatingDoubleTreeMap();

    static {
        offAxisCompMap.put(0.0, 0.0);
        offAxisCompMap.put(45.0, 0.005);
        offAxisCompMap.put(90.0, 0.008);
    }

    /** Table to compensate for the shooter curving left
     * Key: distance (meters), Value: robot angle offset (degrees, WPI CCW+)
     */
    private static final InterpolatingDoubleTreeMap robotAngleCompMap = new InterpolatingDoubleTreeMap();

    static {
        robotAngleCompMap.put(0.0, 0.0);
        robotAngleCompMap.put(1.0, -8.0);
        robotAngleCompMap.put(5.0, -5.0);
    }
    
    @AutoLogOutput @Setter @Getter protected double shotCompensationRotations = 0.001;
    
    public void adjustShotCompensationRotations(double deltaRotations) {
        shotCompensationRotations += deltaRotations;
    }
    
    private static RobotState instance;
    
    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }
    
    // Pose Estimation Members
    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
    TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    // Odometry
    private final SwerveDriveKinematics kinematics;
    private SwerveDriveWheelPositions lastWheelPositions =
    new SwerveDriveWheelPositions(
    new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    });
    private Rotation2d lastGyroAngle = new Rotation2d();
    private Twist2d robotVelocity = new Twist2d();
    
    /** Cached latest aiming parameters. Calculated in {@code getAimingParameters()} */
    private AimingParameters latestParameters = null;

    private Rotation2d latestMoonshotAngle = null;
    
    private RobotState() {
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(Constants.kOdometryStateStdDevs.get(i, 0), 2));
        }
        kinematics = Constants.kKinematics;
    }
    
    /** Add odometry observation */
    public void addOdometryObservation(OdometryObservation observation) {
        latestParameters = null;
        latestMoonshotAngle = null;
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        // Check gyro connected
        if (observation.gyroAngle != null) {
            // Update dtheta for twist if gyro connected
            twist =
            new Twist2d(
            twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
            lastGyroAngle = observation.gyroAngle();
        }
        // Add twist to odometry pose
        odometryPose = odometryPose.exp(twist);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(twist);
    }
    
    public void addVisionObservation(VisionObservation observation) {
        latestParameters = null;
        latestMoonshotAngle = null;
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
            > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }
        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            // exit if not there
            return;
        }
        
        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
        
        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }
        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
        // scale transform by visionK
        var kTimesTransform =
        visionK.times(
        VecBuilder.fill(
        transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform =
        new Transform2d(
        kTimesTransform.get(0, 0),
        kTimesTransform.get(1, 0),
        Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
        
        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }
    
    public void addVelocityData(Twist2d robotVelocity) {
        latestParameters = null;
        latestMoonshotAngle = null;
        this.robotVelocity = robotVelocity;
    }

    public Rotation2d getTargetAngleForMoonshot() {
        if (latestMoonshotAngle != null) {
            return latestMoonshotAngle;
        }

        Transform2d fieldToTarget = 
        GeomUtil.toTransform2d(
            AllianceFlipUtil.apply(
                FieldConstants.ampCenter.plus(
                    new Translation2d(-0.5, -0.5)
                )
            )
        );

        Pose2d fieldToPredictedVehicle = getPredictedPose(lookahead.get(), lookahead.get());
        Pose2d fieldToPredictedVehicleFixed =
        new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d(Math.PI));
        
        // Translation2d predictedVehicleToTargetTranslation =
        // GeomUtil.inverse(fieldToPredictedVehicle).transformBy(fieldToTarget).getTranslation();
        Translation2d predictedVehicleFixedToTargetTranslation =
        GeomUtil.inverse(fieldToPredictedVehicleFixed).transformBy(fieldToTarget).getTranslation();
        
        Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();

        latestMoonshotAngle = targetVehicleDirection;

        Logger.recordOutput("RobotState/Moonshot/TargetDirection", latestMoonshotAngle.getDegrees());

        return latestMoonshotAngle;
    }
    
    public AimingParameters getAimingParameters() {
        if (latestParameters != null) {
            // Cache previously calculated aiming parameters. Cache is invalidated whenever new
            // observations are added.
            return latestParameters;
        }
        
        // Transform2d fieldToTarget =
        //     AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
        //         .toTranslation2d()
        //         .toTransform2d();
        
        Transform2d fieldToTarget =
        GeomUtil.toTransform2d(
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
        .toTranslation2d()
        );
        Pose2d fieldToPredictedVehicle = getPredictedPose(lookahead.get(), lookahead.get());
        Pose2d fieldToPredictedVehicleFixed =
        new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d(Math.PI));
        
        Translation2d predictedVehicleToTargetTranslation =
        GeomUtil.inverse(fieldToPredictedVehicle).transformBy(fieldToTarget).getTranslation();
        Translation2d predictedVehicleFixedToTargetTranslation =
        GeomUtil.inverse(fieldToPredictedVehicleFixed).transformBy(fieldToTarget).getTranslation();
        
        Rotation2d vehicleToGoalDirection = predictedVehicleToTargetTranslation.getAngle();
        
        Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
        double targetDistance = predictedVehicleToTargetTranslation.getNorm();
        
        double feedVelocity =
        robotVelocity.dx * vehicleToGoalDirection.getSin() / targetDistance
        - robotVelocity.dy * vehicleToGoalDirection.getCos() / targetDistance;

        double degreesOffAxis = Math.abs(targetVehicleDirection.minus(fieldToTarget.getRotation()).getDegrees());

        targetVehicleDirection = targetVehicleDirection.plus(Rotation2d.fromDegrees(robotAngleCompMap.get(targetDistance)));

        latestParameters =
        new AimingParameters(
        targetVehicleDirection,
        Rotation2d.fromRotations(
        tiltAngleMap.get(targetDistance) + offAxisCompMap.get(degreesOffAxis) + shotCompensationRotations),
        targetDistance,
        feedVelocity,
        degreesOffAxis);

        Logger.recordOutput("RobotState/AimingParameters/DriveHeading", latestParameters.driveHeading());
        Logger.recordOutput("RobotState/AimingParameters/ShooterTilt", latestParameters.tiltAngle());
        Logger.recordOutput("RobotState/AimingParameters/EffectiveDistance", latestParameters.effectiveDistance());
        Logger.recordOutput("RobotState/AimingParameters/DriveFeedVelocity", latestParameters.driveFeedVelocity());
        Logger.recordOutput("RobotState/AimingParameters/DegreesOffAxis", latestParameters.degreesOffAxis());

        return latestParameters;
    }
    
    /**
    * Reset estimated pose and odometry pose to pose <br>
    * Clear pose buffer
    */
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        odometryPose = initialPose;
        poseBuffer.clear();
    }
    
    @AutoLogOutput(key = "RobotState/FieldVelocity")
    public Twist2d fieldVelocity() {
        Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
        return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
    }
    
    public Translation2d linearVelocity() {
        return new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    }
    
    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }
    
    /**
    * Predicts what our pose will be in the future. Allows separate translation and rotation
    * lookaheads to account for varying latencies in the different measurements.
    *
    * @param translationLookaheadS The lookahead time for the translation of the robot
    * @param rotationLookaheadS The lookahead time for the rotation of the robot
    * @return The predicted pose.
    */
    public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
        return getEstimatedPose()
        .exp(
        new Twist2d(
        robotVelocity.dx * translationLookaheadS,
        robotVelocity.dy * translationLookaheadS,
        robotVelocity.dtheta * rotationLookaheadS));
    }
    
    @AutoLogOutput(key = "RobotState/OdometryPose")
    public Pose2d getOdometryPose() {
        return odometryPose;
    }
}