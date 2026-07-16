package edu.ftcphoenix.robots.phoenix.autonomous;

import org.junit.Test;

import java.util.Collections;

import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused alliance filtering and readiness checks for Phoenix Auto profile snapshots. */
public final class PhoenixAutoProfilesTest {

    @Test
    public void profileForRetainsOneIndependentSelectedAllianceTarget() {
        PhoenixProfile base = PhoenixProfile.defaults();
        PhoenixProfile.AutoAimConfig.ScoringTarget sourceRed =
                base.autoAim.scoringTargets.get(base.auto.redAllianceScoringTagId);
        PhoenixAutoSpec red = spec(PhoenixAutoSpec.Alliance.RED);

        PhoenixProfile result = PhoenixAutoProfiles.profileFor(red, base);

        assertEquals(
                Collections.singleton(base.auto.redAllianceScoringTagId),
                result.autoAim.scoringTargets.keySet()
        );
        assertEquals(2, base.autoAim.scoringTargets.size());
        PhoenixProfile.AutoAimConfig.ScoringTarget copiedRed =
                result.autoAim.scoringTargets.get(base.auto.redAllianceScoringTagId);
        assertNotSame(sourceRed, copiedRed);
        assertNotSame(sourceRed.aimOffset, copiedRed.aimOffset);

        copiedRed.label = "Auto-only label";
        copiedRed.aimOffset.forwardInches = 7.5;
        assertFalse("Auto-only label".equals(sourceRed.label));
        assertFalse(sourceRed.aimOffset.forwardInches == 7.5);
    }

    @Test
    public void inactiveAllianceTargetIsNotRequired() {
        PhoenixProfile base = PhoenixProfile.defaults();
        base.autoAim.scoringTargets.remove(base.auto.blueAllianceScoringTagId);

        PhoenixProfile result = PhoenixAutoProfiles.profileFor(
                spec(PhoenixAutoSpec.Alliance.RED),
                base
        );

        assertEquals(
                Collections.singleton(base.auto.redAllianceScoringTagId),
                result.autoAim.scoringTargets.keySet()
        );
    }

    @Test
    public void missingSelectedTargetFailsInsteadOfSilentlyKeepingTheCatalog() {
        PhoenixProfile base = PhoenixProfile.defaults();
        int selectedTagId = base.auto.redAllianceScoringTagId;
        base.autoAim.scoringTargets.remove(selectedTagId);

        try {
            PhoenixAutoProfiles.profileFor(spec(PhoenixAutoSpec.Alliance.RED), base);
            fail("Expected the missing selected target to fail readiness");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("[auto.selected_target_missing]"));
            assertTrue(expected.getMessage().contains(Integer.toString(selectedTagId)));
            assertTrue(expected.getMessage().contains("redAllianceScoringTagId"));
            assertTrue(expected.getMessage().contains("Fix:"));
        }
    }

    @Test
    public void selectedTargetMustAlsoBeAFixedFieldFact() {
        PhoenixProfile base = PhoenixProfile.defaults();
        int selectedTagId = base.auto.redAllianceScoringTagId;
        base.field.fixedAprilTagLayout = new SimpleTagLayout()
                .add(20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        try {
            PhoenixAutoProfiles.profileFor(spec(PhoenixAutoSpec.Alliance.RED), base);
            fail("Expected a target absent from the fixed layout to fail readiness");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("[auto.selected_target_not_fixed]"));
            assertTrue(expected.getMessage().contains(Integer.toString(selectedTagId)));
            assertTrue(expected.getMessage().contains("fixedAprilTagLayout"));
        }
    }

    @Test
    public void blueSelectionUsesOnlyTheConfiguredBlueTarget() {
        PhoenixProfile base = PhoenixProfile.defaults();
        PhoenixProfile result = PhoenixAutoProfiles.profileFor(
                spec(PhoenixAutoSpec.Alliance.BLUE),
                base
        );

        assertEquals(
                Collections.singleton(base.auto.blueAllianceScoringTagId),
                result.autoAim.scoringTargets.keySet()
        );
        assertTrue(base.autoAim.scoringTargets.containsKey(base.auto.redAllianceScoringTagId));
        assertTrue(base.autoAim.scoringTargets.containsKey(base.auto.blueAllianceScoringTagId));
    }

    private static PhoenixAutoSpec spec(PhoenixAutoSpec.Alliance alliance) {
        return PhoenixAutoSpec.builder()
                .alliance(alliance)
                .strategy(PhoenixAutoStrategyId.SAFE_PRELOAD)
                .build();
    }
}
