package edu.ftcphoenix.robots.phoenix;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused policy and immutability checks for Phoenix readiness reports. */
public final class PhoenixReadinessTest {

    @Test
    public void teleOpPoseAssistsRequireBothCalibrationAcknowledgements() {
        PhoenixProfile profile = calibratedProfile();
        assertTrue(PhoenixReadiness.teleOpPoseAssists(profile.calibration).isAllowed());

        profile.calibration.pinpointPodOffsetsCalibrated = false;
        PhoenixReadiness.Result missingOffsets =
                PhoenixReadiness.teleOpPoseAssists(profile.calibration);
        assertFalse(missingOffsets.isAllowed());
        assertEquals(
                Arrays.asList("pose.pinpoint_pod_offsets_uncalibrated"),
                issueIds(missingOffsets)
        );

        profile.calibration.pinpointPodOffsetsCalibrated = true;
        profile.calibration.pinpointAxesVerified = false;
        PhoenixReadiness.Result missingAxes =
                PhoenixReadiness.teleOpPoseAssists(profile.calibration);
        assertFalse(missingAxes.isAllowed());
        assertEquals(
                Arrays.asList("pose.pinpoint_axes_unverified"),
                issueIds(missingAxes)
        );
    }

    @Test
    public void explicitPedroTestAllowsButClearlyWarnsAboutRelaxedChecks() {
        PhoenixProfile profile = calibratedProfile();
        profile.calibration.pinpointPodOffsetsCalibrated = false;
        PhoenixAutoSpec spec = spec(PhoenixAlliance.RED,
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST);

        PhoenixReadiness.Result result = pedroAuto(
                spec, profile, PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );

        assertTrue(result.isAllowed());
        assertTrue(result.hasWarnings());
        assertNull(result.firstBlockingIssueOrNull());
        assertEquals(
                Arrays.asList(
                        "auto.pedro_integration_test",
                        "pose.pinpoint_pod_offsets_uncalibrated",
                        "auto.route_integration_only"
                ),
                issueIds(result)
        );
        for (PhoenixReadiness.Issue issue : result.issues()) {
            assertEquals(PhoenixReadiness.Severity.WARNING, issue.severity());
        }
    }

    @Test
    public void unverifiedAxesBlockEvenTheExplicitPedroTest() {
        PhoenixProfile profile = calibratedProfile();
        profile.calibration.pinpointAxesVerified = false;
        PhoenixAutoSpec spec = spec(PhoenixAlliance.BLUE,
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST);

        PhoenixReadiness.Result result = pedroAuto(
                spec, profile, PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );

        assertFalse(result.isAllowed());
        assertEquals("pose.pinpoint_axes_unverified", result.firstBlockingIssueOrNull().id());
        assertEquals(
                Arrays.asList(
                        "auto.pedro_integration_test",
                        "pose.pinpoint_axes_unverified",
                        "auto.route_integration_only"
                ),
                issueIds(result)
        );
    }

    @Test
    public void matchAutoBlocksUncalibratedOffsetsAndIntegrationOnlyGeometry() {
        PhoenixProfile profile = calibratedProfile();
        profile.calibration.pinpointPodOffsetsCalibrated = false;
        PhoenixAutoSpec spec = spec(PhoenixAlliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);

        PhoenixReadiness.Result result = pedroAuto(
                spec, profile, PhoenixReadiness.AutoPurpose.MATCH_AUTO
        );

        assertFalse(result.isAllowed());
        assertFalse(result.hasWarnings());
        assertEquals(
                Arrays.asList(
                        "pose.pinpoint_pod_offsets_uncalibrated",
                        "auto.route_not_match_ready"
                ),
                issueIds(result)
        );
    }

    @Test
    public void DriverStationPurposeMustMatchTheSelectedStrategy() {
        PhoenixProfile profile = calibratedProfile();
        PhoenixAutoSpec ordinary = spec(PhoenixAlliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);
        PhoenixAutoSpec integration = spec(PhoenixAlliance.RED,
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST);

        PhoenixReadiness.Result ordinaryFromTestEntry = pedroAuto(
                ordinary, profile, PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        PhoenixReadiness.Result integrationFromMatchEntry = pedroAuto(
                integration, profile, PhoenixReadiness.AutoPurpose.MATCH_AUTO
        );

        assertEquals(
                "auto.purpose_strategy_mismatch",
                ordinaryFromTestEntry.firstBlockingIssueOrNull().id()
        );
        assertTrue(ordinaryFromTestEntry.hasWarnings());
        assertEquals(
                "auto.purpose_strategy_mismatch",
                integrationFromMatchEntry.firstBlockingIssueOrNull().id()
        );
        assertFalse(integrationFromMatchEntry.hasWarnings());
    }

    @Test
    public void everyCurrentStrategyIsCheckedUnderBothDriverStationPurposes() {
        PhoenixProfile profile = calibratedProfile();

        for (PhoenixAutoStrategyId strategy : PhoenixAutoStrategyId.values()) {
            PhoenixAutoSpec spec = spec(PhoenixAlliance.BLUE, strategy);
            for (PhoenixReadiness.AutoPurpose purpose : PhoenixReadiness.AutoPurpose.values()) {
                PhoenixReadiness.Result result = pedroAuto(spec, profile, purpose);

                if (purpose == PhoenixReadiness.AutoPurpose.MATCH_AUTO) {
                    assertFalse(strategy + " must not arm integration-only match geometry",
                            result.isAllowed());
                    assertTrue(hasIssue(result, "auto.route_not_match_ready"));
                } else {
                    assertTrue(hasIssue(result, "auto.pedro_integration_test"));
                    assertTrue(hasIssue(result, "auto.route_integration_only"));
                    assertEquals(
                            strategy == PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST,
                            result.isAllowed()
                    );
                }
            }
        }
    }

    @Test
    public void allianceScoringTagMappingIsModeNeutral() {
        PhoenixProfile profile = calibratedProfile();
        profile.targeting.redAllianceScoringTagId = 41;
        profile.targeting.blueAllianceScoringTagId = 42;

        assertEquals(41, profile.targeting.scoringTagIdFor(PhoenixAlliance.RED));
        assertEquals(42, profile.targeting.scoringTagIdFor(PhoenixAlliance.BLUE));
    }

    @Test
    public void selectedAllianceFactsIgnoreTheInactiveAlliance() {
        PhoenixProfile profile = calibratedProfile();
        profile.targeting.scoringTargets.remove(profile.targeting.blueAllianceScoringTagId);
        profile.fixedAprilTagLayout = new SimpleTagLayout()
                .add(
                        profile.targeting.redAllianceScoringTagId,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0
                );
        PhoenixReadiness.Result result = allianceTarget(PhoenixAlliance.RED, profile);

        assertTrue(result.isAllowed());
        assertTrue(result.issues().isEmpty());
    }

    @Test
    public void selectedAllianceTargetMustExistInCatalogAndFixedLayout() {
        PhoenixProfile missingCatalogTarget = calibratedProfile();
        missingCatalogTarget.targeting.scoringTargets.remove(
                missingCatalogTarget.targeting.redAllianceScoringTagId
        );

        PhoenixReadiness.Result catalogResult = allianceTarget(
                PhoenixAlliance.RED, missingCatalogTarget
        );
        assertEquals(
                Arrays.asList("targeting.selected_scoring_tag_missing"),
                issueIds(catalogResult)
        );

        PhoenixProfile missingFixedTarget = calibratedProfile();
        missingFixedTarget.fixedAprilTagLayout = new SimpleTagLayout()
                .add(20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        PhoenixReadiness.Result layoutResult = allianceTarget(
                PhoenixAlliance.RED, missingFixedTarget
        );
        assertEquals(
                Arrays.asList("targeting.selected_scoring_tag_not_fixed"),
                issueIds(layoutResult)
        );
    }

    @Test
    public void allianceTargetErrorsNameTheExactSharedProfileField() {
        PhoenixProfile redInvalid = calibratedProfile();
        redInvalid.targeting.redAllianceScoringTagId = -1;

        PhoenixReadiness.Result redResult = allianceTarget(PhoenixAlliance.RED, redInvalid);
        assertEquals(
                Arrays.asList("targeting.selected_scoring_tag_id_invalid"),
                issueIds(redResult)
        );
        assertIssueNamesField(
                redResult.issues().get(0),
                "PhoenixProfile.targeting.redAllianceScoringTagId"
        );

        PhoenixProfile blueInvalid = calibratedProfile();
        blueInvalid.targeting.blueAllianceScoringTagId = -2;
        PhoenixReadiness.Result blueResult = allianceTarget(PhoenixAlliance.BLUE, blueInvalid);
        assertEquals(
                Arrays.asList("targeting.selected_scoring_tag_id_invalid"),
                issueIds(blueResult)
        );
        assertIssueNamesField(
                blueResult.issues().get(0),
                "PhoenixProfile.targeting.blueAllianceScoringTagId"
        );
    }

    @Test
    public void reportIsAnImmutableSnapshotWithDeterministicErrors() {
        PhoenixProfile profile = calibratedProfile();
        profile.calibration.pinpointAxesVerified = false;
        profile.calibration.pinpointPodOffsetsCalibrated = false;
        PhoenixAutoSpec spec = spec(PhoenixAlliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);

        PhoenixReadiness.Result result = pedroAuto(
                spec, profile, PhoenixReadiness.AutoPurpose.MATCH_AUTO
        );
        List<String> expectedIds = Arrays.asList(
                "pose.pinpoint_axes_unverified",
                "pose.pinpoint_pod_offsets_uncalibrated",
                "auto.route_not_match_ready"
        );
        assertEquals(expectedIds, issueIds(result));

        profile.calibration.pinpointAxesVerified = true;
        profile.calibration.pinpointPodOffsetsCalibrated = true;
        assertEquals(expectedIds, issueIds(result));

        try {
            result.issues().clear();
            fail("Expected the readiness issue list to be immutable");
        } catch (UnsupportedOperationException expected) {
            // Expected.
        }

        try {
            result.requireAllowed("Phoenix match Auto");
            fail("Expected blocking readiness issues to throw");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("Phoenix match Auto is not ready"));
            for (String id : expectedIds) {
                assertTrue(expected.getMessage().contains("[" + id + "]"));
            }
            assertTrue(expected.getMessage().contains("Fix:"));
        }
    }

    private static PhoenixProfile calibratedProfile() {
        PhoenixProfile profile = PhoenixProfile.current();
        profile.calibration.pinpointAxesVerified = true;
        profile.calibration.pinpointPodOffsetsCalibrated = true;
        return profile;
    }

    private static PhoenixAutoSpec spec(PhoenixAlliance alliance,
                                        PhoenixAutoStrategyId strategy) {
        return PhoenixAutoSpec.builder()
                .alliance(alliance)
                .strategy(strategy)
                .build();
    }

    private static PhoenixReadiness.Result pedroAuto(
            PhoenixAutoSpec spec,
            PhoenixProfile profile,
            PhoenixReadiness.AutoPurpose purpose
    ) {
        return PhoenixReadiness.pedroAuto(
                spec,
                profile.calibration,
                profile.targeting,
                profile.fixedAprilTagLayout,
                purpose
        );
    }

    private static PhoenixReadiness.Result allianceTarget(
            PhoenixAlliance alliance,
            PhoenixProfile profile
    ) {
        return PhoenixReadiness.allianceScoringTarget(
                alliance,
                profile.targeting,
                profile.fixedAprilTagLayout
        );
    }

    private static List<String> issueIds(PhoenixReadiness.Result result) {
        java.util.ArrayList<String> ids = new java.util.ArrayList<String>();
        for (PhoenixReadiness.Issue issue : result.issues()) {
            ids.add(issue.id());
        }
        return ids;
    }

    private static boolean hasIssue(PhoenixReadiness.Result result, String id) {
        for (PhoenixReadiness.Issue issue : result.issues()) {
            if (id.equals(issue.id())) {
                return true;
            }
        }
        return false;
    }

    private static void assertIssueNamesField(PhoenixReadiness.Issue issue, String fieldName) {
        assertTrue(issue.message().contains(fieldName));
        assertTrue(issue.remediation().contains(fieldName));
    }
}
