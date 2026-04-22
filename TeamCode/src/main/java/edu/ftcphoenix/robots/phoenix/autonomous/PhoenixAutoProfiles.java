package edu.ftcphoenix.robots.phoenix.autonomous;

import java.util.LinkedHashMap;
import java.util.Objects;

import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Applies autonomous setup choices to a Phoenix profile before the robot container is constructed.
 *
 * <p>Phoenix targeting reads its configured scoring-target catalog during robot construction. That
 * means alliance-specific target filtering belongs here, before {@code PhoenixRobot} is created,
 * rather than inside an OpMode loop or deep inside the robot container.</p>
 */
public final class PhoenixAutoProfiles {

    /**
     * Default blue scoring tag id in the checked-in Decode target catalog.
     */
    public static final int DEFAULT_BLUE_SCORING_TAG_ID = 20;

    /**
     * Default red scoring tag id in the checked-in Decode target catalog.
     */
    public static final int DEFAULT_RED_SCORING_TAG_ID = 24;

    private PhoenixAutoProfiles() {
        // Utility class.
    }

    /**
     * Create an Auto-specific profile snapshot from a base profile and selected Auto spec.
     *
     * @param spec selected autonomous setup
     * @param base base profile to copy; when null, {@link PhoenixProfile#current()} is used
     * @return copied profile with autonomous-specific edits applied
     */
    public static PhoenixProfile profileFor(PhoenixAutoSpec spec, PhoenixProfile base) {
        Objects.requireNonNull(spec, "spec");
        PhoenixProfile profile = (base == null ? PhoenixProfile.current() : base).copy();

        switch (spec.alliance) {
            case RED:
                keepOnlyScoringTagIfPresent(profile, DEFAULT_RED_SCORING_TAG_ID);
                break;
            case BLUE:
                keepOnlyScoringTagIfPresent(profile, DEFAULT_BLUE_SCORING_TAG_ID);
                break;
            default:
                break;
        }

        return profile;
    }

    private static void keepOnlyScoringTagIfPresent(PhoenixProfile profile, int tagId) {
        PhoenixProfile.AutoAimConfig.ScoringTarget target = profile.autoAim.scoringTargets.get(tagId);
        if (target == null) {
            // Leave the catalog intact rather than producing an Auto profile with no target choices.
            return;
        }

        LinkedHashMap<Integer, PhoenixProfile.AutoAimConfig.ScoringTarget> filtered =
                new LinkedHashMap<Integer, PhoenixProfile.AutoAimConfig.ScoringTarget>();
        filtered.put(tagId, target.copy());
        profile.autoAim.scoringTargets = filtered;
    }
}
