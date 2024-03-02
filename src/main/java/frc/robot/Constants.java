package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.dashboard.Alert;
import frc.robot.lib.dashboard.Alert.AlertType;
import frc.robot.lib.drive.SwerveSetpointGenerator.KinematicLimits;
import frc.robot.lib.motion.MotionProfileConstraints;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {
    private static final RobotType robot = RobotType.ROBOT_2024_HARD_ROCK;
    public static final double loopPeriodSecs = 0.02;
    public static final boolean tuningMode = true;
    
    public static boolean invalidRobotAlertSent = false;
    
    public static RobotType getRobot() {
        if (!disableHAL && RobotBase.isReal()) {
            if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
                if (!invalidRobotAlertSent) {
                    new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
                    .set(true);
                    invalidRobotAlertSent = true;
                }
                return RobotType.ROBOT_2023_HEAVYMETAL;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }
    
    public static Mode getMode() {
        switch (getRobot()) {
            case ROBOT_2023_HEAVYMETAL:
            case ROBOT_2023_FLAPJACK:
            return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            
            case ROBOT_SIMBOT:
            return Mode.SIM;
            
            default:
            return Mode.REAL;
        }
    }
    
    public static final Map<RobotType, String> logFolders =
    Map.of(RobotType.ROBOT_2023_FLAPJACK, "/home/lvuser", RobotType.ROBOT_2023_HEAVYMETAL, "/home/lvuser"); // log to internal storage
    // Map.of(RobotType.ROBOT_2023_CN1, "/media/sda2", RobotType.ROBOT_2023_CN2, "/media/sda2/"); // log to sd card
    
    public static enum RobotType {
        ROBOT_2024_HARD_ROCK,
        /**
         * 2023 "Heavy Metal" Chassis
         */
        ROBOT_2023_HEAVYMETAL,

        /**
         * 2023 "Flapjack" Chassis (Formerly A-Frame)
         */
        ROBOT_2023_FLAPJACK,

        /**
         * Robot Simulator
         */
        ROBOT_SIMBOT
    }
    
    public static enum Mode {
        REAL,
        REPLAY,
        SIM
    }
    
    // Function to disable HAL interaction when running without native libs
    public static boolean disableHAL = false;
    
    public static void disableHAL() {
        disableHAL = true;
    }
    
    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final String kRioCANBusName = "rio";
    public static final String kCANivoreCANBusName = "canivore";
    public static final double kCancoderBootAllowanceSeconds = 10.0;
    
    // Drive constants
    public static final double kSDS_L2 = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
    public static final double kSDS_L3 = 1 / ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0));
    public static final double kSDS_L3_BOOSTED = 1 / ((16.0 / 50.0) * (26.0 / 16.0) * (15.0 / 45.0));
    public static final double kDriveReduction = kSDS_L3;//(14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kDriveWheelDiameter = Units.inchesToMeters(4.0); // meters
    public static final double kDriveTrackwidthMeters = Units.inchesToMeters(23.25);
    public static final double kDriveWheelbaseMeters = Units.inchesToMeters(23.25);
    
    public static final double kMaxVelocityMetersPerSecond = 5.05; //Calibrated 3/12 on Comp Bot
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.4;

    public static final boolean kDriveUseOpenLoop = false;

    // switch from field-oriented commutation to trapezoidal commutation to boost top speed
    public static final boolean kUseFieldWeakening = true;
    public static final double kMinVelocityForFieldWeakening = 4.0;
    
    // Robot constants
    public static final double kMaxDriveAcceleration = 1867 * 0.8;   // m/s^2 tuned 2/18 practice bot
    public static final double kTrackScrubFactor = 1;
    public static final double kDriveTeleopAngleBiasFactor = 0.125; // compensates for velocity vector deviation while rotating
    
    
    /**
    * The maximum angular velocity of the robot in radians per second.
    * <p>
    * This is a measure of how fast the robot can rotate in place.
    */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double kMaxAngularVelocityRadiansPerSecond = 11.386413;
    
    public static final KinematicLimits kUncappedKinematicLimits = new KinematicLimits();
    static {
        kUncappedKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kUncappedKinematicLimits.kMaxSteeringVelocity = Double.MAX_VALUE;
    }
    
    public static final KinematicLimits kAzimuthOnlyKinematicLimits = new KinematicLimits();
    static {
        kAzimuthOnlyKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kAzimuthOnlyKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kAzimuthOnlyKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }
    
    public static final KinematicLimits kTeleopKinematicLimits = new KinematicLimits();
    static {
        kTeleopKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kTeleopKinematicLimits.kMaxDriveAcceleration = kTeleopKinematicLimits.kMaxDriveVelocity / 0.1;
        kTeleopKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }
    
    public static final KinematicLimits kFastKinematicLimits = new KinematicLimits();
    static {
        kFastKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kFastKinematicLimits.kMaxDriveAcceleration = kFastKinematicLimits.kMaxDriveVelocity / 0.2;
        kFastKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1000.0);
    }
    
    public static final KinematicLimits kSmoothKinematicLimits = new KinematicLimits();
    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond * .7;
        kSmoothKinematicLimits.kMaxDriveAcceleration = kSmoothKinematicLimits.kMaxDriveVelocity / 1.0;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }
    
    // public static final Translation2d[] kWheelPositions = {
    //     // Front left
    //     new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
    //     // Front right
    //     new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0),
    //     // Back left
    //     new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
    //     // Back right
    //     new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0)
    // };

    public static final Translation2d[] kWheelPositions = {
        // Front left
        new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
        // Front right
        new Translation2d(Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0),
        // Back left
        new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
        // Back right
        new Translation2d(-Constants.kDriveTrackwidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0)
    };
    
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        kWheelPositions
    );

    // TODO rename these to be steer/drive
    public static final double kMk4AziMMKp = 6.000; //0.505
    public static final double kMk4AziMMKi = 0;
    public static final double kMk4AziMMKd = 0;//0.0004; //0.2 / 500
    public static final double kMk4AziMMKs = 0.8;
    public static final double kMk4AziKv = 0.1224;

    public static final double kMk4AziMMCruiseVelocity = 98.0;
    public static final double kMk4AziMMAccel = 1000.0;


    public static final double kMk4AziPositionKp = 1.0005; //0.505
    public static final double kMk4AziPositionKi = 0;
    public static final double kMk4AziPositionKd = 0.0004; //0.2 / 500

    public static final double kMk4DriveVelocityKp = 0.02 * 12;
    public static final double kMk4DriveVelocityKi = 0.0;
    public static final double kMk4DriveVelocityKd = 0.000002 * 12; //0.01
    public static final double kMk4DriveVelocityKv = 1 / 101.98 * 12;// (kMaxVelocityMetersPerSecond / (Math.PI * Constants.kDriveWheelDiameter * Constants.kDriveReduction));
    public static final double kMk4DriveVelocityKs = 0.8;

    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxVelocityMetersPerSecond /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = kMaxAccelerationMetersPerSecondSquared /
            Math.hypot(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final TrapezoidProfile.Constraints kPositionControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxVelocityMetersPerSecond, kMaxVelocityMetersPerSecond);

    public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
            0.8 * Constants.kMaxVelocityMetersPerSecond,
            0.8 * -Constants.kMaxVelocityMetersPerSecond,
            0.8 * Constants.kMaxAccelerationMetersPerSecondSquared);
    public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
            0.5 * Constants.kMaxAngularSpeedRadiansPerSecond,
            0.5 * -Constants.kMaxAngularSpeedRadiansPerSecond,
            1.0 * Constants.kMaxAngularAccelerationRadiansPerSecondSquared);


    
    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.5; // degree error
    public static final double kSwerveHeadingControllerMaintainThreshold = 25.0; // at what error will the heading controller switch from snap mode to maintain mode

    public static final double kSnapSwerveHeadingKp = 0.05;
    public static final double kSnapSwerveHeadingKi = 0.0;
    public static final double kSnapSwerveHeadingKd = 0.02;

    public static final double kMaintainSwerveHeadingKpHighVelocity = 0.0125; //0.0225
    public static final double kMaintainSwerveHeadingKiHighVelocity = 0.0;
    public static final double kMaintainSwerveHeadingKdHighVelocity = 0.001; // 0.003

    public static final double kMaintainSwerveHeadingKpLowVelocity = 0.02;  // 0.01;
    public static final double kMaintainSwerveHeadingKiLowVelocity = 0.0;
    public static final double kMaintainSwerveHeadingKdLowVelocity = 0.0;

    // Swerve heading controller gains
    public static final double kHeadingControllerKp = 2.54;
    public static final double kHeadingControllerKi = 0.0;
    public static final double kHeadingControllerKd = 0.0;
    public static final double kHeadingControllerKffv = 1.0;
    public static final double kHeadingControllerKffa = 0.0;
    public static final double kHeadingControllerKs = 0.0;

    public static final double kSnapRadiusKp = 2.0;
    public static final double kSnapRadiusKi = 0.0;
    public static final double kSnapRadiusKd = 0.0;

    public static final double kMaintainRadiusKp = 1.5;
    public static final double kMaintainRadiusKi = 0.0;
    public static final double kMaintainRadiusKd = 0.0;
    
    //Pure Pursuit Constants
    public static final double kPathLookaheadTime = 0.25; // From 1323 (2019)
    public static final double kPathMinLookaheadDistance = 12.0; //From 1323 (2019)
    public static final double kAdaptivePathMinLookaheadDistance = 6.0;
    public static final double kAdaptivePathMaxLookaheadDistance = 24.0;
    public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

    //Auton Driving
    public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(2.5, 0, 0, 0),
        new PIDConstants(2.5, 0, 0, 0),
        kMaxVelocityMetersPerSecond,
        kWheelPositions[0].getNorm(),
        new ReplanningConfig()
    );

    //Auto-Align
    public static final double kAutoAlignAllowableDistance = 2.0; //Meters


    //Heading Controller Drive
    public static final double kFeedforwardScaleFactor = 2.0;

    public static final class DriveSubsystem {
        public static final Slot0Configs kDrivePIDConfig = new Slot0Configs();
        static {
            kDrivePIDConfig.kP = 0;//0.2 * 12; 
            kDrivePIDConfig.kI = 0.0;
            kDrivePIDConfig.kD = 0.0;//00002 * 12;
            kDrivePIDConfig.kV = 0.117;// (kMaxVelocityMetersPerSecond / (Math.PI * Constants.kDriveWheelDiameter * Constants.kDriveReduction));;
            kDrivePIDConfig.kS = 0.0;//8;
        }

        public static final Slot0Configs kSteerPIDConfig = new Slot0Configs();
        static {
            kSteerPIDConfig.kP = 3.000 * 2 * Math.PI;
            kSteerPIDConfig.kI = 0.0;
            kSteerPIDConfig.kD = 0.0 * 2 * Math.PI;
            kSteerPIDConfig.kV = 0.1224 * 2 * Math.PI;
            kSteerPIDConfig.kS = 0.8;
        }

        public static final MotionMagicConfigs kSteerMagicConfig = new MotionMagicConfigs();
        static {
            kSteerMagicConfig.MotionMagicCruiseVelocity = 98.0 / (2 * Math.PI);
            kSteerMagicConfig.MotionMagicAcceleration = 1000.0 / (2 * Math.PI);
            kSteerMagicConfig.MotionMagicJerk = 0.0;
        }

    }

    public static final class VisionSubsystem {
        public static final String kCameraName = "limelight";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class ArmSubsystem {
        public static final class J1 {
            public static final int kMasterMotorID = 30;
            public static final int kFollowerMotorID = 31;
            public static final String kMotorBus = "canivore"; // Both motors must be on same bus to use follower mode

            public static final boolean invertMaster = false;
            public static final boolean invertFollower = false;

            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            
            public static final double kMagicVel = 1.0;
            public static final double kMagicAccel = 2.0;
            public static final double kMagicJerk = 0.0;

            public static final double kLiberalAllowableError = 0.05;
            public static final double kConservativeAllowableError = 0.01;
            
            public static final double kHomePosition = 0.0;
            public static final double kMinTargetPosition = 0.0;
            public static final double kMaxTargetPosition = 0.0;

            public static final double kAbsoluteMaxPosition = 0.0;
            public static final double kAbsoluteMinPosition = 0.0;
        }

        public static final class J2 {
            public static final int kMasterMotorID = 32;
            public static final String kMotorBus = "canivore"; // Both motors must be on same bus to use follower mode

            public static final boolean invertMaster = false;
            
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kMagicVel = 1.5;
            public static final double kMagicAccel = 3.0;
            public static final double kMagicJerk = 0.0;

            public static final double kLiberalAllowableError = 0.05;
            public static final double kConservativeAllowableError = 0.01;

            public static final double kHomePosition = 0.4;
            public static final double kMinTargetPosition = 0.0;
            public static final double kMaxTargetPosition = 0.0;

            public static final double kAbsoluteMaxPosition = 0.0;
            public static final double kAbsoluteMinPosition = 0.0;
        }
    }

    public static final class Intake {
            public static final int kMasterMotorID = 33;
            public static final String kMotorBus = "canivore";

            public static final boolean invertMaster = false;

            public static final double kIdleThrottle = 0.0;
            public static final double kPartialIntakeThrottle = 0.5;
            public static final double kHandoffThrottle = 0.7;
            public static final double kConstantThrottle = 0.7;
            public static final double kReverseThrottle = -0.33;
        }

    public static final class Indexer {
        public static final int kMotorID = 40;
        public static final String kMotorBus = "canivore";

        public static final double kIdleThrottle = 0.0;
        public static final double kIntakingThrottle = 1.0;
        public static final double kOverfedThrottle = -0.1;
        public static final double kScoringThrottle = 1.0;
        public static final double kReversingThrottle = -0.5;
    }

    public static final class ShooterFlywheels {
        public static final int kLeftMotorID = 42;
        public static final int kRightMotorID = 43;
        public static final String kMotorBus = "canivore";

        public static final double kS = 0.0;
        public static final double kV = 0.11;
        public static final double kA = 0.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kMaxRPSForBrakeMode = 8;
        public static final double kMaxRPSForIdleControl = 20;
        public static final double kIdleRPS = 15.0;
        public static final double kRPSTolerance = 5;
    }

    public static final class ShooterTilt {
        public static final int kMotorID = 48;
        public static final String kMotorBus = "canivore";

        public static final boolean invertMaster = false;
            
        public static final double kG = 0.35;
        public static final double kS = 2.5;
        public static final double kV = 40.0;
        public static final double kA = 20.0;
        public static final double kP = 64.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kMagicVel = 0.5;
        public static final double kMagicAccel = 5;
        public static final double kMagicJerk = 10.0;

        public static final double kLiberalAllowableError = 0.01;
        public static final double kConservativeAllowableError = 0.005;

        public static final double kHomePosition = 0.0;
        public static final double kMinTargetPosition = 0.0;
        public static final double kMaxTargetPosition = 0.1;

        public static final double kAbsoluteMaxPosition = 0.1;
        public static final double kAbsoluteMinPosition = 0.0;
    }

    public static final int kMaxLEDCount = 127;
    
    /** Checks whether the robot the correct robot is selected when deploying. */
    public static void main(String... args) {
        if (robot == RobotType.ROBOT_SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
            System.exit(1);
        }
    }

    public static final double kLEDClosenessDeadbandMeters = 0.03;
}