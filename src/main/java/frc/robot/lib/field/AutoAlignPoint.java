package frc.robot.lib.field;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoAlignPoint {
    public enum Points {
        SPEAKER_FRONT_CLOSE(new AutoAlignPoint(new Pose2d(1.331, 5.561, Rotation2d.fromRotations(0.0)), 2.0)),
        SPEAKER_FRONT_PODIUM(new AutoAlignPoint(new Pose2d(2.849, 4.565, Rotation2d.fromRotations(-0.052)), 2.0)),
        AMP_SCORING(new AutoAlignPoint(new Pose2d(1.901, 7.440, Rotation2d.fromRotations(-0.25)), 2.0)), // 7.740
        SOURCE_LEFT_INTAKE(new AutoAlignPoint(new Pose2d(15.984, 1.263, Rotation2d.fromRotations(0.336)), 4.0)),
        SOURCE_RIGHT_INTAKE(new AutoAlignPoint(new Pose2d(14.993, 0.641, Rotation2d.fromRotations(0.336)), 4.0)),
        STAGE_LEFT_CLIMB(new AutoAlignPoint(new Pose2d(0, 0, Rotation2d.fromRotations(0)), 2.0)),
        STAGE_CENTER_CLIMB(new AutoAlignPoint(new Pose2d(0, 0, Rotation2d.fromRotations(0)), 2.0)),
        STAGE_RIGHT_CLIMB(new AutoAlignPoint(new Pose2d(0, 0, Rotation2d.fromRotations(0)), 2.0));

        public AutoAlignPoint point;

        Points(AutoAlignPoint point) {
            this.point = point;
        }
    }

    private Pose2d kPose;
    private double kMaxDistanceToAlign;

    public AutoAlignPoint(Pose2d pose, double maxDistance) {
        kPose = pose;
        kMaxDistanceToAlign = maxDistance;
    }

    public Optional<Pose2d> getPose(Pose2d currentPose) {
        var pose = forceGetPose();
        double dist = currentPose.relativeTo(pose).getTranslation().getNorm();
        if (dist <= kMaxDistanceToAlign) {
            return Optional.of(pose);
        }
        return Optional.empty();
    }

    public Pose2d forceGetPose() {
        return kPose;
    }
}
