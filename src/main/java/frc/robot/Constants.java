package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
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
    private static final RobotType robot = RobotType.ROBOT_2024_SONIC;
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
                return RobotType.ROBOT_2024_SONIC;
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
            case ROBOT_2024_SONIC:
            return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            
            case ROBOT_SIMBOT:
            return Mode.SIM;
            
            default:
            return Mode.REAL;
        }
    }
    
    public static final Map<RobotType, String> logFolders =
        Map.of(
            RobotType.ROBOT_2024_SONIC, "/media/sda2"
        ); // log to internal storage
    // Map.of(RobotType.ROBOT_2023_CN1, "/media/sda2", RobotType.ROBOT_2023_CN2, "/media/sda2/"); // log to sd card
    
    public static enum RobotType {
        ROBOT_2024_SONIC,
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

    public static final double kSnapSwerveHeadingKp = 0.025;
    public static final double kSnapSwerveHeadingKi = 0.0;
    public static final double kSnapSwerveHeadingKd = 0.0005;

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

    //Auton Driving
    public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(2.5, 0, 0, 0),
        new PIDConstants(2.5, 0, 0, 0),
        kMaxVelocityMetersPerSecond,
        kWheelPositions[0].getNorm(),
        new ReplanningConfig()
    );

    public static final Matrix<N3, N1> kOdometryStateStdDevs =
      switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      };

    //Auto-Align
    public static final double kAutoAlignAllowableDistance = 2.0; //Meters

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