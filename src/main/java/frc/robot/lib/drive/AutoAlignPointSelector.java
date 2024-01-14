package frc.robot.lib.drive;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.field.AutoAlignPoint;
import frc.robot.lib.field.AutoAlignPoint.Points;

public class AutoAlignPointSelector {

    public enum RequestedAlignment {
        AUTO,

        SPEAKER_AUTO,
        SPEAKER_CLOSE,
        SPEAKER_PODIUM,

        AMP,

        SOURCE_AUTO,
        SOURCE_LEFT,
        SOURCE_RIGHT,

        STAGE_AUTO,
        STAGE_LEFT,
        STAGE_CENTER,
        STAGE_RIGHT
    }

    private static Optional<Pose2d> minimizeDistance(Pose2d from, Pose2d[] to) {
        if (to.length == 0) {
            return Optional.empty();
        }
        double closestDistance = Integer.MAX_VALUE;
        Pose2d closestPose = to[0];
        for (int i = 0; i < to.length; i++) {
            double distance = from.relativeTo(to[i]).getTranslation().getNorm();
            if (distance <  closestDistance) {
                closestDistance = distance;
                closestPose = to[i];
            }
        }
        return Optional.of(closestPose);
    }

    private static Optional<Pose2d> minimizeDistanceUsingEnums(Pose2d from, AutoAlignPoint.Points... to) {
        if (to.length == 0) {
            return Optional.empty();
        }

        ArrayList<Pose2d> poses = new ArrayList<>();
        for (AutoAlignPoint.Points alignPoint : to) {
            var pose = alignPoint.point.getPose(from);
            if (pose.isPresent()) {
                poses.add(pose.get());
            }
        }

        return minimizeDistance(from, poses.toArray(new Pose2d[poses.size()]));
    }

    public static Optional<Pose2d> getAlignTarget(Pose2d currentPose, RequestedAlignment alignment) {
        var alliance = DriverStation.getAlliance();
    
        
        if (alliance.isEmpty()) {
            return Optional.empty();
        }

        switch (alignment) {
            case AUTO:
                return minimizeDistanceUsingEnums(currentPose, Points.SOURCE_LEFT_INTAKE, Points.SOURCE_RIGHT_INTAKE, Points.AMP_SCORING, Points.SPEAKER_FRONT_CLOSE, Points.SPEAKER_FRONT_PODIUM);
            case SPEAKER_AUTO:
                return minimizeDistanceUsingEnums(
                    currentPose, 
                    Points.SPEAKER_FRONT_CLOSE, 
                    Points.SPEAKER_FRONT_PODIUM
                );
            case SPEAKER_CLOSE:
                return Points.SPEAKER_FRONT_CLOSE.point.getPose(currentPose);
            case SPEAKER_PODIUM:
                return Points.SPEAKER_FRONT_PODIUM.point.getPose(currentPose);
            case AMP:
                return Points.AMP_SCORING.point.getPose(currentPose);
            case SOURCE_AUTO:
                return minimizeDistanceUsingEnums(
                    currentPose,
                    Points.SOURCE_LEFT_INTAKE,
                    Points.SOURCE_RIGHT_INTAKE
                );
            case SOURCE_LEFT:
                return Points.SOURCE_LEFT_INTAKE.point.getPose(currentPose);
            case SOURCE_RIGHT:
                return Points.SOURCE_RIGHT_INTAKE.point.getPose(currentPose);
            case STAGE_AUTO:
                return minimizeDistanceUsingEnums(
                    currentPose, 
                    Points.STAGE_LEFT_CLIMB, 
                    Points.STAGE_CENTER_CLIMB,
                    Points.STAGE_RIGHT_CLIMB
                );
            case STAGE_LEFT:
                return Points.STAGE_LEFT_CLIMB.point.getPose(currentPose);
            case STAGE_CENTER:
                return Points.STAGE_CENTER_CLIMB.point.getPose(currentPose);
            case STAGE_RIGHT:
                return Points.STAGE_RIGHT_CLIMB.point.getPose(currentPose);
            default:
                return Optional.empty();
        }
    }
}
