package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.fail;

/** Proves the complete authoring contract for immutable Drive Guidance tuning. */
public final class DriveGuidanceTuningTest {

    private static final AbsolutePoseEstimator ESTIMATOR = new AbsolutePoseEstimator() {
        @Override
        public void update(LoopClock clock) {
            // Builder-only test dependency; no runtime evaluation is performed.
        }

        @Override
        public PoseEstimate getEstimate() {
            return null;
        }
    };

    @Test
    public void defaultsRemainUnchanged() {
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        assertEquals(0.05, tuning.kPTranslate, 0.0);
        assertEquals(0.60, tuning.maxTranslateCmd, 0.0);
        assertEquals(2.50, tuning.kPAim, 0.0);
        assertEquals(0.80, tuning.maxOmegaCmd, 0.0);
        assertEquals(0.00, tuning.minOmegaCmd, 0.0);
        assertEquals(Math.toRadians(1.0), tuning.aimDeadbandRad, 0.0);
    }

    @Test
    public void gainsRejectEveryNonFiniteClassAndFiniteNegativeValues() {
        for (double value : nonFiniteValues()) {
            assertInvalid(
                    "DriveGuidancePlan.Tuning.kPTranslate must be finite and >= 0; received "
                            + value,
                    () -> DriveGuidancePlan.Tuning.defaults().withTranslateKp(value)
            );
            assertInvalid(
                    "DriveGuidancePlan.Tuning.kPAim must be finite and >= 0; received " + value,
                    () -> DriveGuidancePlan.Tuning.defaults().withAimKp(value)
            );
        }

        assertInvalid(
                "DriveGuidancePlan.Tuning.kPTranslate must be finite and >= 0; received -0.1",
                () -> DriveGuidancePlan.Tuning.defaults().withTranslateKp(-0.1)
        );
        assertInvalid(
                "DriveGuidancePlan.Tuning.kPAim must be finite and >= 0; received -0.1",
                () -> DriveGuidancePlan.Tuning.defaults().withAimKp(-0.1)
        );
    }

    @Test
    public void commandCapsRejectEveryNonFiniteAndFiniteOutOfRangeClass() {
        for (double value : nonFiniteValues()) {
            assertNormalizedFieldInvalid("maxTranslateCmd", value,
                    () -> DriveGuidancePlan.Tuning.defaults().withMaxTranslateCmd(value));
            assertNormalizedFieldInvalid("maxOmegaCmd", value,
                    () -> DriveGuidancePlan.Tuning.defaults().withMaxOmegaCmd(value));
        }

        for (double value : new double[]{-0.1, 1.1}) {
            assertNormalizedFieldInvalid("maxTranslateCmd", value,
                    () -> DriveGuidancePlan.Tuning.defaults().withMaxTranslateCmd(value));
            assertNormalizedFieldInvalid("maxOmegaCmd", value,
                    () -> DriveGuidancePlan.Tuning.defaults().withMaxOmegaCmd(value));
        }
    }

    @Test
    public void minimumOmegaRejectsNonFiniteNegativeAndContradictoryTuples() {
        for (double value : nonFiniteValues()) {
            assertInvalid(
                    "DriveGuidancePlan.Tuning.minOmegaCmd must be finite and >= 0; received "
                            + value,
                    () -> DriveGuidancePlan.Tuning.defaults().withMinOmegaCmd(value)
            );
        }
        assertInvalid(
                "DriveGuidancePlan.Tuning.minOmegaCmd must be finite and >= 0; received -0.1",
                () -> DriveGuidancePlan.Tuning.defaults().withMinOmegaCmd(-0.1)
        );

        assertInvalid(
                "DriveGuidancePlan.Tuning.minOmegaCmd/maxOmegaCmd must satisfy "
                        + "minOmegaCmd <= maxOmegaCmd; received minOmegaCmd=0.5, maxOmegaCmd=0.4",
                () -> new DriveGuidancePlan.Tuning(0.1, 0.5, 0.2, 0.4, 0.5, 0.1)
        );
        assertInvalid(
                "DriveGuidancePlan.Tuning.minOmegaCmd/kPAim requires kPAim > 0 when "
                        + "minOmegaCmd > 0; received minOmegaCmd=0.1, kPAim=0.0",
                () -> new DriveGuidancePlan.Tuning(0.1, 0.5, 0.0, 0.4, 0.1, 0.1)
        );

        DriveGuidancePlan.Tuning withMinimum = DriveGuidancePlan.Tuning.defaults()
                .withMinOmegaCmd(0.2);
        assertInvalid(
                "DriveGuidancePlan.Tuning.minOmegaCmd/maxOmegaCmd must satisfy "
                        + "minOmegaCmd <= maxOmegaCmd; received minOmegaCmd=0.2, maxOmegaCmd=0.1",
                () -> withMinimum.withMaxOmegaCmd(0.1)
        );
        assertInvalid(
                "DriveGuidancePlan.Tuning.minOmegaCmd/kPAim requires kPAim > 0 when "
                        + "minOmegaCmd > 0; received minOmegaCmd=0.2, kPAim=0.0",
                () -> withMinimum.withAimKp(0.0)
        );
    }

    @Test
    public void aimDeadbandRejectsEveryNonFiniteAndFiniteOutOfRangeClass() {
        for (double value : nonFiniteValues()) {
            assertDeadbandInvalid(value);
        }
        assertDeadbandInvalid(-0.1);
        assertDeadbandInvalid(Math.nextUp(Math.PI));
    }

    @Test
    public void allDocumentedBoundariesAreAccepted() {
        DriveGuidancePlan.Tuning disabled = new DriveGuidancePlan.Tuning(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
        );
        assertEquals(0.0, disabled.kPTranslate, 0.0);
        assertEquals(0.0, disabled.maxTranslateCmd, 0.0);
        assertEquals(0.0, disabled.kPAim, 0.0);
        assertEquals(0.0, disabled.maxOmegaCmd, 0.0);
        assertEquals(0.0, disabled.minOmegaCmd, 0.0);
        assertEquals(0.0, disabled.aimDeadbandRad, 0.0);

        DriveGuidancePlan.Tuning extremes = new DriveGuidancePlan.Tuning(
                Double.MAX_VALUE,
                1.0,
                Double.MAX_VALUE,
                1.0,
                1.0,
                Math.PI
        );
        assertEquals(Double.MAX_VALUE, extremes.kPTranslate, 0.0);
        assertEquals(1.0, extremes.maxTranslateCmd, 0.0);
        assertEquals(Double.MAX_VALUE, extremes.kPAim, 0.0);
        assertEquals(1.0, extremes.maxOmegaCmd, 0.0);
        assertEquals(1.0, extremes.minOmegaCmd, 0.0);
        assertEquals(Math.PI, extremes.aimDeadbandRad, 0.0);
    }

    @Test
    public void eachWitherCreatesAnIndependentCompleteValue() {
        DriveGuidancePlan.Tuning defaults = DriveGuidancePlan.Tuning.defaults();

        assertOnlyFieldChanged(defaults, defaults.withTranslateKp(0.2), 0, 0.2);
        assertOnlyFieldChanged(defaults, defaults.withMaxTranslateCmd(0.3), 1, 0.3);
        assertOnlyFieldChanged(defaults, defaults.withAimKp(3.0), 2, 3.0);
        assertOnlyFieldChanged(defaults, defaults.withMaxOmegaCmd(0.4), 3, 0.4);
        assertOnlyFieldChanged(defaults, defaults.withMinOmegaCmd(0.1), 4, 0.1);
        assertOnlyFieldChanged(defaults, defaults.withAimDeadbandRad(0.2), 5, 0.2);

        assertEquals(0.05, defaults.kPTranslate, 0.0);
        assertEquals(0.60, defaults.maxTranslateCmd, 0.0);
        assertEquals(2.50, defaults.kPAim, 0.0);
        assertEquals(0.80, defaults.maxOmegaCmd, 0.0);
        assertEquals(0.00, defaults.minOmegaCmd, 0.0);
        assertEquals(Math.toRadians(1.0), defaults.aimDeadbandRad, 0.0);
    }

    @Test
    public void completePlanBuilderUsesValidatedWithersAndRetainsAnswers() {
        DriveGuidancePlan plan = DriveGuidance.plan()
                .translateTo()
                .fieldPointInches(12.0, 8.0)
                .solveWith()
                .localizationOnlyWithDefaults(ESTIMATOR)
                .driveTuning()
                .translateKp(0.2)
                .maxTranslateCmd(0.3)
                .aimKp(3.0)
                .maxOmegaCmd(0.4)
                .minOmegaCmd(0.1)
                .aimDeadbandRad(0.2)
                .doneDriveTuning()
                .build();

        assertTuning(plan.tuning, 0.2, 0.3, 3.0, 0.4, 0.1, 0.2);

        DriveGuidance.DriveTuningBranch branch = completePlanTuningBranch();
        assertInvalid(
                "DriveGuidancePlan.Tuning.kPTranslate must be finite and >= 0; received NaN",
                () -> branch.translateKp(Double.NaN)
        );
    }

    @Test
    public void planFromSpecBuilderUsesValidatedWithersAndRetainsAnswers() {
        DriveGuidanceSpec spec = translationSpec();
        DriveGuidancePlan plan = DriveGuidance.plan(spec)
                .driveTuning()
                .translateKp(0.15)
                .maxTranslateCmd(0.25)
                .aimKp(2.0)
                .maxOmegaCmd(0.5)
                .minOmegaCmd(0.05)
                .aimDeadbandRad(0.1)
                .doneDriveTuning()
                .build();

        assertTuning(plan.tuning, 0.15, 0.25, 2.0, 0.5, 0.05, 0.1);

        DriveGuidance.DriveTuningBranch branch = DriveGuidance.plan(spec).driveTuning();
        assertInvalid(
                "DriveGuidancePlan.Tuning.maxTranslateCmd must be finite and in [0, 1]; "
                        + "received 1.1",
                () -> branch.maxTranslateCmd(1.1)
        );
    }

    private static DriveGuidance.DriveTuningBranch completePlanTuningBranch() {
        return DriveGuidance.plan()
                .translateTo()
                .fieldPointInches(12.0, 8.0)
                .solveWith()
                .localizationOnlyWithDefaults(ESTIMATOR)
                .driveTuning();
    }

    private static DriveGuidanceSpec translationSpec() {
        return DriveGuidance.spec()
                .translateTo()
                .fieldPointInches(12.0, 8.0)
                .solveWith()
                .localizationOnlyWithDefaults(ESTIMATOR)
                .build();
    }

    private static double[] nonFiniteValues() {
        return new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
    }

    private static void assertNormalizedFieldInvalid(String field,
                                                     double value,
                                                     ThrowingRunnable action) {
        assertInvalid(
                "DriveGuidancePlan.Tuning." + field
                        + " must be finite and in [0, 1]; received " + value,
                action
        );
    }

    private static void assertDeadbandInvalid(double value) {
        assertInvalid(
                "DriveGuidancePlan.Tuning.aimDeadbandRad must be finite and in [0, Math.PI]; "
                        + "received " + value,
                () -> DriveGuidancePlan.Tuning.defaults().withAimDeadbandRad(value)
        );
    }

    private static void assertOnlyFieldChanged(DriveGuidancePlan.Tuning original,
                                               DriveGuidancePlan.Tuning changed,
                                               int changedIndex,
                                               double expectedChangedValue) {
        assertNotSame(original, changed);
        double[] originalValues = valuesOf(original);
        double[] changedValues = valuesOf(changed);
        for (int i = 0; i < originalValues.length; i++) {
            assertEquals(
                    i == changedIndex ? expectedChangedValue : originalValues[i],
                    changedValues[i],
                    0.0
            );
        }
    }

    private static double[] valuesOf(DriveGuidancePlan.Tuning tuning) {
        return new double[]{
                tuning.kPTranslate,
                tuning.maxTranslateCmd,
                tuning.kPAim,
                tuning.maxOmegaCmd,
                tuning.minOmegaCmd,
                tuning.aimDeadbandRad
        };
    }

    private static void assertTuning(DriveGuidancePlan.Tuning tuning,
                                     double kPTranslate,
                                     double maxTranslateCmd,
                                     double kPAim,
                                     double maxOmegaCmd,
                                     double minOmegaCmd,
                                     double aimDeadbandRad) {
        assertEquals(kPTranslate, tuning.kPTranslate, 0.0);
        assertEquals(maxTranslateCmd, tuning.maxTranslateCmd, 0.0);
        assertEquals(kPAim, tuning.kPAim, 0.0);
        assertEquals(maxOmegaCmd, tuning.maxOmegaCmd, 0.0);
        assertEquals(minOmegaCmd, tuning.minOmegaCmd, 0.0);
        assertEquals(aimDeadbandRad, tuning.aimDeadbandRad, 0.0);
    }

    private static void assertInvalid(String expectedMessage, ThrowingRunnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException: " + expectedMessage);
        } catch (IllegalArgumentException expected) {
            assertEquals(expectedMessage, expected.getMessage());
        }
    }

    private interface ThrowingRunnable {
        void run();
    }
}
