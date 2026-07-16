package edu.ftcphoenix.robots.phoenix.autonomous;

import java.util.LinkedHashMap;
import java.util.Objects;

import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixReadiness;

/**
 * Applies autonomous setup choices to a Phoenix profile before the robot container is constructed.
 *
 * <p>Phoenix targeting reads its configured scoring-target catalog during robot construction. That
 * means alliance-specific target filtering belongs here, before {@code PhoenixRobot} is created,
 * rather than inside an OpMode loop or deep inside the robot container.</p>
 *
 * <p>The alliance-to-tag mapping itself stays in {@link PhoenixProfile.AutoConfig}. This utility
 * applies the selected {@link PhoenixAutoSpec}; it does not own field constants.</p>
 */
public final class PhoenixAutoProfiles {

    private PhoenixAutoProfiles() {
        // Utility class.
    }

    /**
     * Create an Auto-specific profile snapshot from a base profile and selected Auto spec.
     *
     * @param spec selected autonomous setup
     * @param base base profile to copy; when null, {@link PhoenixProfile#current()} is used
     * @return copied profile with autonomous-specific edits applied
     * @throws IllegalStateException when the selected alliance scoring tag is absent from either
     *                               the target catalog or the fixed-field layout
     */
    public static PhoenixProfile profileFor(PhoenixAutoSpec spec, PhoenixProfile base) {
        Objects.requireNonNull(spec, "spec");
        PhoenixProfile source = base == null ? PhoenixProfile.current() : base;
        PhoenixReadiness.autoProfile(spec, source).requireAllowed("Phoenix Auto profile");
        PhoenixProfile profile = source.copy();

        switch (spec.alliance) {
            case RED:
                keepOnlyScoringTag(profile, profile.auto.redAllianceScoringTagId);
                break;
            case BLUE:
                keepOnlyScoringTag(profile, profile.auto.blueAllianceScoringTagId);
                break;
            default:
                break;
        }

        return profile;
    }

    private static void keepOnlyScoringTag(PhoenixProfile profile, int tagId) {
        PhoenixProfile.AutoAimConfig.ScoringTarget target = profile.autoAim.scoringTargets.get(tagId);
        LinkedHashMap<Integer, PhoenixProfile.AutoAimConfig.ScoringTarget> filtered =
                new LinkedHashMap<Integer, PhoenixProfile.AutoAimConfig.ScoringTarget>();
        filtered.put(tagId, target.copy());
        profile.autoAim.scoringTargets = filtered;
    }
}
