package edu.ftcsushi.fw.localization.apriltag;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/**
 * Real solver checks showing that two pose representations can agree under shared calibration error.
 *
 * <p>The robot is independently authored at field (0,0), yaw zero. Three tag coordinates and the
 * camera's translation are explicit scalar facts. Observation-provided poses are synthetic inputs,
 * not FTC SDK computations; these tests make no camera, vendor, or physical accuracy claim.</p>
 */
public final class FixedTagFieldPoseSolverSharedEvidenceTest {
    private static final double EPS = 1e-9;
    private static final double[][] TRUE_TAGS = {{24, -6}, {30, 0}, {24, 6}};

    @Test
    public void commonLayoutShiftCanPassAgreementWhileBothAlternativesMissTruth() {
        List<AprilTagObservation> observations = observations(0, 0, new Pose3d(3.25, 4, 0, 0, 0, 0));
        SimpleTagLayout shifted = layout(3, 4);
        FixedTagFieldPoseSolver.Result preferred = solve(true, observations, shifted, CameraMountConfig.identity());
        FixedTagFieldPoseSolver.Result geometry = solve(false, observations, shifted, CameraMountConfig.identity());

        // The independent geometry answer is layout shift minus unchanged relative observation.
        // The supplied-pose alternative carries that same error plus a quarter-inch perturbation.
        assertPose(geometry, 3, 4);
        assertPose(preferred, 3.25, 4);
        assertSelected(observations, preferred, true);
        assertSelected(observations, geometry, false);
        assertEquals(0.25, distance(preferred, geometry), EPS);
        assertEquals(5.0, errorFromIndependentOrigin(geometry), EPS);
        assertEquals(Math.hypot(3.25, 4), errorFromIndependentOrigin(preferred), EPS);
        assertEquals("agreement chooses one pose per tag; it does not add a second observation",
                3, preferred.candidateCount);
        assertEquals(geometry.totalWeight, preferred.totalWeight, EPS);
        assertEquals(geometry.quality, preferred.quality, EPS);
        assertTrue("consistent wrong results can retain a large solver score", preferred.quality > 0.75);

        FixedTagFieldPoseSolver.Result control = solve(true,
                observations(0, 0, Pose3d.zero()), layout(0, 0), CameraMountConfig.identity());
        assertPose(control, 0, 0);
        assertEquals("the correct reference changes error, not within-frame consistency",
                control.quality, preferred.quality, EPS);
    }

    @Test
    public void commonMountErrorCanAlsoPassAgreementWithoutIndependentAccuracy() {
        // Actual camera is two inches forward and one left. Both interpretations are supplied
        // a mount three inches farther forward and four farther left than that physical fact.
        List<AprilTagObservation> observations = observations(2, 1, new Pose3d(-3.25, -4, 0, 0, 0, 0));
        CameraMountConfig wrongMount = CameraMountConfig.of(5, 5, 0, 0, 0, 0);
        FixedTagFieldPoseSolver.Result preferred = solve(true, observations, layout(0, 0), wrongMount);
        FixedTagFieldPoseSolver.Result geometry = solve(false, observations, layout(0, 0), wrongMount);

        assertPose(geometry, -3, -4);
        assertPose(preferred, -3.25, -4);
        assertSelected(observations, preferred, true);
        assertSelected(observations, geometry, false);
        assertEquals(0.25, distance(preferred, geometry), EPS);
        assertEquals(5.0, errorFromIndependentOrigin(geometry), EPS);
        assertTrue(errorFromIndependentOrigin(preferred) > 5.0);
        assertEquals(geometry.quality, preferred.quality, EPS);

        FixedTagFieldPoseSolver.Result control = solve(true,
                observations(2, 1, Pose3d.zero()), layout(0, 0), CameraMountConfig.of(2, 1, 0, 0, 0, 0));
        assertPose(control, 0, 0);
        assertEquals(control.quality, preferred.quality, EPS);
    }

    @Test
    public void disagreementGateSelectsGeometryButDoesNotVerifyTheSharedMap() {
        List<AprilTagObservation> observations = observations(0, 0, new Pose3d(30, 40, 0, 0, 0, 0));
        FixedTagFieldPoseSolver.Result result = solve(true, observations, layout(3, 4), CameraMountConfig.identity());

        // The supplied alternative disagrees by45in, beyond the default8in gate. Rejecting that
        // alternative cannot correct the remaining map's independently known (3,4)in error.
        assertPose(result, 3, 4);
        assertSelected(observations, result, false);
        assertEquals(5.0, errorFromIndependentOrigin(result), EPS);
        assertEquals(3, result.candidateCount);
        assertEquals(3, result.acceptedCount);
    }

    private static FixedTagFieldPoseSolver.Result solve(boolean preferSupplied,
                                                        List<AprilTagObservation> observations,
                                                        SimpleTagLayout layout,
                                                        CameraMountConfig mount) {
        FixedTagFieldPoseSolver.Config config = FixedTagFieldPoseSolver.Config.defaults();
        config.preferObservationFieldPose = preferSupplied;
        return new FixedTagFieldPoseSolver(config).solve(observations, layout, mount);
    }

    /** Explicit observation geometry from the authored camera translation, with no pose chaining. */
    private static List<AprilTagObservation> observations(double actualCameraX, double actualCameraY,
                                                        Pose3d suppliedFieldPose) {
        List<AprilTagObservation> result = new ArrayList<>();
        for (int i = 0; i < TRUE_TAGS.length; i++) {
            Pose3d observed = new Pose3d(TRUE_TAGS[i][0] - actualCameraX,
                    TRUE_TAGS[i][1] - actualCameraY, 0, 0, 0, 0);
            result.add(AprilTagObservation.target(i + 1, observed, suppliedFieldPose));
        }
        return result;
    }

    private static SimpleTagLayout layout(double commonShiftX, double commonShiftY) {
        SimpleTagLayout result = new SimpleTagLayout();
        for (int i = 0; i < TRUE_TAGS.length; i++)
            result.addPose(i + 1, new Pose3d(TRUE_TAGS[i][0] + commonShiftX,
                    TRUE_TAGS[i][1] + commonShiftY, 0, 0, 0, 0));
        return result;
    }

    private static void assertSelected(List<AprilTagObservation> observations,
                                       FixedTagFieldPoseSolver.Result result,
                                       boolean supplied) {
        assertEquals(3, result.acceptedContributions.size());
        for (int i = 0; i < observations.size(); i++) {
            FixedTagFieldPoseSolver.Contribution contribution = result.acceptedContributions.get(i);
            assertSame(observations.get(i), contribution.observation);
            if (supplied) assertTrue(contribution.usedObservationFieldPose);
            else assertFalse(contribution.usedObservationFieldPose);
        }
    }

    private static void assertPose(FixedTagFieldPoseSolver.Result result, double x, double y) {
        assertTrue(result.hasPose);
        assertEquals(x, result.fieldToRobotPose.xInches, EPS);
        assertEquals(y, result.fieldToRobotPose.yInches, EPS);
        assertEquals(0.0, result.fieldToRobotPose.yawRad, EPS);
        assertEquals(3, result.acceptedCount);
    }

    private static double distance(FixedTagFieldPoseSolver.Result a, FixedTagFieldPoseSolver.Result b) {
        return Math.hypot(a.fieldToRobotPose.xInches - b.fieldToRobotPose.xInches,
                a.fieldToRobotPose.yInches - b.fieldToRobotPose.yInches);
    }

    private static double errorFromIndependentOrigin(FixedTagFieldPoseSolver.Result result) {
        return Math.hypot(result.fieldToRobotPose.xInches, result.fieldToRobotPose.yInches);
    }
}
