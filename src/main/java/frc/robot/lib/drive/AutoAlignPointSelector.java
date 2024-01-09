package frc.robot.lib.drive;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.lib.field.AprilTag;
import frc.robot.lib.field.Field;

public class AutoAlignPointSelector {


    public enum RequestedAlignment {
        CENTER, // force center (will do nothing if feeder)
        RIGHT, // force right
        LEFT, // force left

        AUTO_CONE, // go to whatever position is closest that can score cubes mid or high
        AUTO_CUBE, // go to whatever position is closest that can score cones mid or high

        AUTO, // go to whatever position is closest
    }

    private static Map<Integer, AprilTag> getTagSet()  {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return Field.Red.kAprilTagMap;
        } else {
            return Field.Blue.kAprilTagMap;
        }
    }

    private static Optional<AprilTag> getNearestTag(Map<Integer, AprilTag> tagMap, Pose2d point) {
        double closestDistance = Integer.MAX_VALUE;
        Optional<AprilTag> closestTag = Optional.empty();
        for (int i : tagMap.keySet()) {
            double distance = tagMap.get(i).getFieldToTag().relativeTo(point).getTranslation().getNorm();
            if (distance <  closestDistance) {
                closestDistance = distance;
                closestTag = Optional.of(tagMap.get(i));
            }
        }
        return closestTag;
    }

    private static Optional<Pose2d> minimizeDistance(Pose2d from, Pose2d[] to) {
        if (to.length == 0) {
            return Optional.empty();
        }
        double closestDistance = Integer.MAX_VALUE;
        Pose2d closestPose = to[0];
        for (int i = 0; i < to.length; i++) {
            double distance = from.relativeTo(to[i]).getTranslation().getNorm();// .distance(to[i]);
            if (distance <  closestDistance) {
                closestDistance = distance;
                closestPose = to[i];
            }
        }
        return Optional.of(closestPose);
    }

    private static Optional<Pose2d> getNearestAlignment(AprilTag tag, Pose2d point, boolean cone, boolean cube) {
        if (tag.isScoring()) {
            Pose2d center = tag.getFieldToTag().transformBy(new Transform2d(tag.getTagToCenterAlign(), new Rotation2d()));
            center = new Pose2d(center.getTranslation(), Rotation2d.fromDegrees(180));
            Pose2d left = tag.getFieldToTag().transformBy(new Transform2d(tag.getTagToLeftAlign(), new Rotation2d()));
            left = new Pose2d(left.getTranslation(), Rotation2d.fromDegrees(180));
            Pose2d right = tag.getFieldToTag().transformBy(new Transform2d(tag.getTagToRightAlign(), new Rotation2d()));
            right = new Pose2d(right.getTranslation(), Rotation2d.fromDegrees(180));
            if (cone && cube) {
                return minimizeDistance(point, new Pose2d[]{left, center, right});
            } else if (cone && !cube) {
                return minimizeDistance(point, new Pose2d[]{left, right});
            } else {
                return Optional.of(center);
            }
        } else {
            Pose2d left = tag.getFieldToTag().transformBy(new Transform2d(tag.getTagToLeftAlign(), new Rotation2d()));
            left = new Pose2d(left.getTranslation(), Rotation2d.fromDegrees(0));
            Pose2d right = tag.getFieldToTag().transformBy(new Transform2d(tag.getTagToRightAlign(), new Rotation2d()));
            right = new Pose2d(right.getTranslation(), Rotation2d.fromDegrees(0));
            return minimizeDistance(point, new Pose2d[]{left,right});
        }
    }

    public static Optional<Pose2d> chooseTargetPoint(Pose2d currentPoint, RequestedAlignment alignment) {
        Map<Integer, AprilTag> mTagMap = getTagSet();
        Optional<AprilTag> closestTag = getNearestTag(mTagMap, currentPoint);
        if (closestTag.isEmpty()) {
            return Optional.empty();
        }
        Optional<Pose2d> targetPose = Optional.empty();
        if (alignment == RequestedAlignment.AUTO || alignment == RequestedAlignment.AUTO_CONE || alignment == RequestedAlignment.AUTO_CUBE) {
            targetPose = getNearestAlignment(closestTag.get(), currentPoint,
                    alignment == RequestedAlignment.AUTO || alignment == RequestedAlignment.AUTO_CONE,
                    alignment == RequestedAlignment.AUTO || alignment == RequestedAlignment.AUTO_CUBE);
        } else {
            if (closestTag.get().isScoring()) {
                switch (alignment) {
                    case CENTER:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(new Transform2d(closestTag.get().getTagToCenterAlign(), new Rotation2d())));
                        break;
                    case LEFT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(new Transform2d(closestTag.get().getTagToLeftAlign(), new Rotation2d())));
                        break;
                    case RIGHT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(new Transform2d(closestTag.get().getTagToRightAlign(), new Rotation2d())));
                        break;
                    default:
                        break;
                }
                targetPose = Optional.of(new Pose2d(targetPose.get().getTranslation(), Rotation2d.fromDegrees(180)));
            } else {
                switch (alignment) {
                    case LEFT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(new Transform2d(closestTag.get().getTagToLeftAlign(), new Rotation2d())));
                        break;
                    case RIGHT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(new Transform2d(closestTag.get().getTagToRightAlign(), new Rotation2d())));
                        break;
                    default:
                        // center align in feeder is useless
                        return Optional.empty();
                }
                targetPose = Optional.of(new Pose2d(targetPose.get().getTranslation(), Rotation2d.fromDegrees(0)));
            }
        }

        if (targetPose.isPresent() && targetPose.get().relativeTo(currentPoint).getTranslation().getNorm() > Constants.kAutoAlignAllowableDistance) {
            return Optional.empty();
        }
        return targetPose;
    }  
}
