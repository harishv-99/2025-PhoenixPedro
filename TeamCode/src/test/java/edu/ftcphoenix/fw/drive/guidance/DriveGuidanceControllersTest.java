package edu.ftcphoenix.fw.drive.guidance;

import org.junit.Test;

import edu.ftcphoenix.fw.drive.DriveSignal;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Regression coverage for bounded Drive Guidance controller math. */
public final class DriveGuidanceControllersTest {

    @Test
    public void defaultTranslationOutputsRemainUnchanged() {
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        DriveSignal belowCap = DriveGuidanceControllers.translationCmd(3.0, -4.0, tuning);
        assertSignal(belowCap, 0.15, -0.20, 0.0, 1e-15);

        DriveSignal capped = DriveGuidanceControllers.translationCmd(12.0, 16.0, tuning);
        assertSignal(capped, 0.36, 0.48, 0.0, 1e-15);
        assertEquals(0.60, Math.hypot(capped.axial, capped.lateral), 1e-15);
    }

    @Test
    public void defaultOmegaOutputsRemainUnchanged() {
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        assertEquals(0.0, DriveGuidanceControllers.omegaCmd(0.01, tuning), 0.0);
        assertEquals(0.5, DriveGuidanceControllers.omegaCmd(0.2, tuning), 0.0);
        assertEquals(0.8, DriveGuidanceControllers.omegaCmd(1.0, tuning), 0.0);
        assertEquals(-0.8, DriveGuidanceControllers.omegaCmd(-1.0, tuning), 0.0);
    }

    @Test
    public void positiveMinimumOmegaSurvivesFiniteGainUnderflow() {
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(Double.MIN_VALUE)
                .withAimDeadbandRad(0.0)
                .withMinOmegaCmd(0.1);

        assertEquals(0.1, DriveGuidanceControllers.omegaCmd(0.5, tuning), 0.0);
        assertEquals(-0.1, DriveGuidanceControllers.omegaCmd(-0.5, tuning), 0.0);
    }

    @Test
    public void zeroGainsDisableTheirChannels() {
        DriveGuidancePlan.Tuning noTranslation = DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(0.0)
                .withMaxTranslateCmd(1.0);
        assertSignal(
                DriveGuidanceControllers.translationCmd(12.0, -7.0, noTranslation),
                0.0,
                0.0,
                0.0,
                0.0
        );

        DriveGuidancePlan.Tuning noAim = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(0.0)
                .withAimDeadbandRad(0.0);
        assertEquals(0.0, DriveGuidanceControllers.omegaCmd(0.5, noAim), 0.0);
    }

    @Test
    public void zeroOmegaCapDisablesTurningAndZeroMinimumAddsNoFloor() {
        DriveGuidancePlan.Tuning cappedAtZero = DriveGuidancePlan.Tuning.defaults()
                .withMaxOmegaCmd(0.0)
                .withAimDeadbandRad(0.0);
        assertEquals(0.0, DriveGuidanceControllers.omegaCmd(0.5, cappedAtZero), 0.0);

        DriveGuidancePlan.Tuning noMinimum = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(0.1)
                .withMaxOmegaCmd(1.0)
                .withMinOmegaCmd(0.0)
                .withAimDeadbandRad(0.0);
        assertEquals(0.001, DriveGuidanceControllers.omegaCmd(0.01, noMinimum), 1e-15);
    }

    @Test
    public void zeroAndPiDeadbandsHonorTheirExactBoundaries() {
        DriveGuidancePlan.Tuning zeroDeadband = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(1.0)
                .withMaxOmegaCmd(1.0)
                .withAimDeadbandRad(0.0);
        assertEquals(0.0, DriveGuidanceControllers.omegaCmd(0.0, zeroDeadband), 0.0);
        assertEquals(0.25, DriveGuidanceControllers.omegaCmd(0.25, zeroDeadband), 0.0);
        assertEquals(-0.25, DriveGuidanceControllers.omegaCmd(-0.25, zeroDeadband), 0.0);

        DriveGuidancePlan.Tuning fullCanonicalDeadband = zeroDeadband
                .withAimDeadbandRad(Math.PI);
        assertEquals(0.0,
                DriveGuidanceControllers.omegaCmd(Math.PI, fullCanonicalDeadband), 0.0);
        assertEquals(0.0,
                DriveGuidanceControllers.omegaCmd(-Math.PI, fullCanonicalDeadband), 0.0);
    }

    @Test
    public void normalizedOneBoundariesRemainTruthful() {
        DriveGuidancePlan.Tuning translation = DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(1.0)
                .withMaxTranslateCmd(1.0);
        DriveSignal command = DriveGuidanceControllers.translationCmd(3.0, 4.0, translation);
        assertSignal(command, 0.6, 0.8, 0.0, 1e-15);
        assertEquals(1.0, Math.hypot(command.axial, command.lateral), 1e-15);

        DriveGuidancePlan.Tuning fullMinimum = DriveGuidancePlan.Tuning.defaults()
                .withMaxOmegaCmd(1.0)
                .withMinOmegaCmd(1.0)
                .withAimDeadbandRad(0.0);
        assertEquals(1.0, DriveGuidanceControllers.omegaCmd(0.01, fullMinimum), 0.0);
        assertEquals(-1.0, DriveGuidanceControllers.omegaCmd(-0.01, fullMinimum), 0.0);
    }

    @Test
    public void maximumFiniteAimGainOverflowStillHonorsOmegaCap() {
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(Double.MAX_VALUE)
                .withMaxOmegaCmd(1.0)
                .withAimDeadbandRad(0.0);

        assertEquals(1.0, DriveGuidanceControllers.omegaCmd(2.0, tuning), 0.0);
        assertEquals(-1.0, DriveGuidanceControllers.omegaCmd(-2.0, tuning), 0.0);
    }

    @Test
    public void exactZeroTranslationCapEmitsExactZero() {
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
                .withMaxTranslateCmd(0.0);

        DriveSignal command = DriveGuidanceControllers.translationCmd(0.5, -0.25, tuning);

        assertEquals(0.0, command.axial, 0.0);
        assertEquals(0.0, command.lateral, 0.0);
        assertEquals(0.0, Math.hypot(command.axial, command.lateral), 0.0);
    }

    @Test
    public void subNanounitTranslationCapIsNeverExceeded() {
        double cap = 1e-12;
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
                .withMaxTranslateCmd(cap);

        DriveSignal command = DriveGuidanceControllers.translationCmd(3.0, -4.0, tuning);

        assertEquals(0.6 * cap, command.axial, 1e-27);
        assertEquals(-0.8 * cap, command.lateral, 1e-27);
        assertTrue(Math.hypot(command.axial, command.lateral) <= cap);
    }

    @Test
    public void maximumFiniteGainOverflowRecoversBoundedDirection() {
        double cap = 0.6;
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(Double.MAX_VALUE)
                .withMaxTranslateCmd(cap);

        DriveSignal command = DriveGuidanceControllers.translationCmd(2.0, -1.0, tuning);

        double expectedUnit = 1.0 / Math.sqrt(5.0);
        assertSignal(
                command,
                2.0 * expectedUnit * cap,
                -expectedUnit * cap,
                0.0,
                1e-15
        );
        assertFiniteAndBounded(command, cap);
    }

    @Test
    public void hugeFiniteVectorMagnitudeOverflowRecoversBoundedDirection() {
        double cap = 0.75;
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(1.0)
                .withMaxTranslateCmd(cap);

        DriveSignal command = DriveGuidanceControllers.translationCmd(
                Double.MAX_VALUE,
                -Double.MAX_VALUE,
                tuning
        );

        double expectedComponent = cap / Math.sqrt(2.0);
        assertSignal(command, expectedComponent, -expectedComponent, 0.0, 1e-15);
        assertFiniteAndBounded(command, cap);
    }

    @Test
    public void maximumFiniteGainWithHugeFiniteVectorStillEmitsFiniteBoundedCommand() {
        double cap = 1.0;
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(Double.MAX_VALUE)
                .withMaxTranslateCmd(cap);

        DriveSignal command = DriveGuidanceControllers.translationCmd(
                -Double.MAX_VALUE,
                Double.MAX_VALUE,
                tuning
        );

        double expectedComponent = cap / Math.sqrt(2.0);
        assertSignal(command, -expectedComponent, expectedComponent, 0.0, 1e-15);
        assertFiniteAndBounded(command, cap);
    }

    private static void assertFiniteAndBounded(DriveSignal command, double cap) {
        assertTrue(Double.isFinite(command.axial));
        assertTrue(Double.isFinite(command.lateral));
        assertTrue(Double.isFinite(command.omega));
        assertTrue(Math.hypot(command.axial, command.lateral) <= cap + 1e-15);
    }

    private static void assertSignal(DriveSignal actual,
                                     double expectedAxial,
                                     double expectedLateral,
                                     double expectedOmega,
                                     double tolerance) {
        assertEquals(expectedAxial, actual.axial, tolerance);
        assertEquals(expectedLateral, actual.lateral, tolerance);
        assertEquals(expectedOmega, actual.omega, tolerance);
    }
}
