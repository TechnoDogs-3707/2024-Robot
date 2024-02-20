package frc.robot.lib;

public class ScoringIntentTracker {
    public enum ScoringIntent {
        NONE,
        INTAKE_GROUND,
        INTAKE_SOURCE,
        SCORE_AMP,
        SCORE_SPEAKER
    }

    private static ScoringIntent scoringIntent = ScoringIntent.NONE;

    public static void setScoringIntent(ScoringIntent scoringIntent) {
        ScoringIntentTracker.scoringIntent = scoringIntent;
    }

    public static ScoringIntent getScoringIntent() {
        return scoringIntent;
    }
}
