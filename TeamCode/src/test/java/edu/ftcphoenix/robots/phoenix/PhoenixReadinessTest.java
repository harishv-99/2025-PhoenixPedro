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
        assertTrue(PhoenixReadiness.teleOpPoseAssists(profile).isAllowed());

        profile.calibration.pinpointPodOffsetsCalibrated = false;
        PhoenixReadiness.Result missingOffsets = PhoenixReadiness.teleOpPoseAssists(profile);
        assertFalse(missingOffsets.isAllowed());
        assertEquals(
                Arrays.asList("pose.pinpoint_pod_offsets_uncalibrated"),
                issueIds(missingOffsets)
        );

        profile.calibration.pinpointPodOffsetsCalibrated = true;
        profile.calibration.pinpointAxesVerified = false;
        PhoenixReadiness.Result missingAxes = PhoenixReadiness.teleOpPoseAssists(profile);
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
        PhoenixAutoSpec spec = spec(PhoenixAutoSpec.Alliance.RED,
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST);

        PhoenixReadiness.Result result = PhoenixReadiness.pedroAuto(
                spec,
                profile,
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
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
        PhoenixAutoSpec spec = spec(PhoenixAutoSpec.Alliance.BLUE,
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST);

        PhoenixReadiness.Result result = PhoenixReadiness.pedroAuto(
                spec,
                profile,
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
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
        PhoenixAutoSpec spec = spec(PhoenixAutoSpec.Alliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);

        PhoenixReadiness.Result result = PhoenixReadiness.pedroAuto(
                spec,
                profile,
                PhoenixReadiness.AutoPurpose.MATCH_AUTO
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
        PhoenixAutoSpec ordinary = spec(PhoenixAutoSpec.Alliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);
        PhoenixAutoSpec integration = spec(PhoenixAutoSpec.Alliance.RED,
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST);

        PhoenixReadiness.Result ordinaryFromTestEntry = PhoenixReadiness.pedroAuto(
                ordinary,
                profile,
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        PhoenixReadiness.Result integrationFromMatchEntry = PhoenixReadiness.pedroAuto(
                integration,
                profile,
                PhoenixReadiness.AutoPurpose.MATCH_AUTO
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
            PhoenixAutoSpec spec = spec(PhoenixAutoSpec.Alliance.BLUE, strategy);
            for (PhoenixReadiness.AutoPurpose purpose : PhoenixReadiness.AutoPurpose.values()) {
                PhoenixReadiness.Result result = PhoenixReadiness.pedroAuto(
                        spec,
                        profile,
                        purpose
                );

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
    public void selectedAllianceFactsIgnoreTheInactiveAlliance() {
        PhoenixProfile profile = calibratedProfile();
        profile.autoAim.scoringTargets.remove(profile.auto.blueAllianceScoringTagId);
        profile.field.fixedAprilTagLayout = new SimpleTagLayout()
                .add(
                        profile.auto.redAllianceScoringTagId,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0
                );
        PhoenixAutoSpec red = spec(PhoenixAutoSpec.Alliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);

        PhoenixReadiness.Result result = PhoenixReadiness.autoProfile(red, profile);

        assertTrue(result.isAllowed());
        assertTrue(result.issues().isEmpty());
    }

    @Test
    public void selectedAllianceTargetMustExistInCatalogAndFixedLayout() {
        PhoenixProfile missingCatalogTarget = calibratedProfile();
        missingCatalogTarget.autoAim.scoringTargets.remove(
                missingCatalogTarget.auto.redAllianceScoringTagId
        );
        PhoenixAutoSpec red = spec(PhoenixAutoSpec.Alliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);

        PhoenixReadiness.Result catalogResult =
                PhoenixReadiness.autoProfile(red, missingCatalogTarget);
        assertEquals(
                Arrays.asList("auto.selected_target_missing"),
                issueIds(catalogResult)
        );

        PhoenixProfile missingFixedTarget = calibratedProfile();
        missingFixedTarget.field.fixedAprilTagLayout = new SimpleTagLayout()
                .add(20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        PhoenixReadiness.Result layoutResult =
                PhoenixReadiness.autoProfile(red, missingFixedTarget);
        assertEquals(
                Arrays.asList("auto.selected_target_not_fixed"),
                issueIds(layoutResult)
        );
    }

    @Test
    public void reportIsAnImmutableSnapshotWithDeterministicErrors() {
        PhoenixProfile profile = calibratedProfile();
        profile.calibration.pinpointAxesVerified = false;
        profile.calibration.pinpointPodOffsetsCalibrated = false;
        PhoenixAutoSpec spec = spec(PhoenixAutoSpec.Alliance.RED,
                PhoenixAutoStrategyId.SAFE_PRELOAD);

        PhoenixReadiness.Result result = PhoenixReadiness.pedroAuto(
                spec,
                profile,
                PhoenixReadiness.AutoPurpose.MATCH_AUTO
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
        PhoenixProfile profile = PhoenixProfile.defaults();
        profile.calibration.pinpointAxesVerified = true;
        profile.calibration.pinpointPodOffsetsCalibrated = true;
        return profile;
    }

    private static PhoenixAutoSpec spec(PhoenixAutoSpec.Alliance alliance,
                                        PhoenixAutoStrategyId strategy) {
        return PhoenixAutoSpec.builder()
                .alliance(alliance)
                .strategy(strategy)
                .build();
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
}
