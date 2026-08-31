package edu.ftcsushi.fw.ftc.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Exhaustive owner-local validation and tagged-resolution dispatch coverage. */
public final class PinpointOdometryPredictorConfigValidationTest {

    @Test
    public void rawCopyPreservesInvalidDraftAndValidatedCopyIsIndependent() {
        PinpointOdometryPredictor.Config source = PinpointOdometryPredictor.Config.defaults();
        source.hardwareMapName = null;
        source.encoderResolution = null;
        source.forwardPodDirection = null;

        PinpointOdometryPredictor.Config raw = source.copy();

        assertNotSame(source, raw);
        assertNull(raw.hardwareMapName);
        assertNull(raw.encoderResolution);
        assertNull(raw.forwardPodDirection);

        source = PinpointOdometryPredictor.Config.defaults();
        source.hardwareMapName = "pinPoint";
        source.quality = 0.61;
        PinpointOdometryPredictor.Config captured = source.validatedCopy("owner.predictor");
        source.hardwareMapName = "later";
        source.quality = 0.1;

        assertEquals("pinPoint", captured.hardwareMapName);
        assertEquals(0.61, captured.quality, 0.0);
        assertSame(source.encoderResolution, captured.encoderResolution);
    }

    @Test
    public void hardwareNameIsRequiredAndContextFallbackIsCanonical() {
        PinpointOdometryPredictor.Config config = PinpointOdometryPredictor.Config.defaults();
        config.hardwareMapName = null;
        RuntimeException nullFailure = capture(() -> config.validatedCopy("robot.localization"));
        assertTrue(nullFailure instanceof NullPointerException);
        assertTrue(nullFailure.getMessage().contains(
                "robot.localization.hardwareMapName must not be null"));

        config.hardwareMapName = " \t ";
        RuntimeException blankFailure = capture(() -> config.validatedCopy("  "));
        assertTrue(blankFailure instanceof IllegalArgumentException);
        assertTrue(blankFailure.getMessage().contains(
                PinpointOdometryPredictor.Config.class.getCanonicalName()
                        + ".hardwareMapName"));
        assertTrue(blankFailure.getMessage().contains("got ' \t '"));

        config.hardwareMapName = " pinPoint ";
        assertEquals(" pinPoint ", config.validatedCopy(null).hardwareMapName);
    }

    @Test
    public void offsetsMustBeFiniteAndSurviveExactMillimetreFloatTranslation() {
        assertInvalidOffsets(Double.NaN, "must be finite inches");
        assertInvalidOffsets(Double.POSITIVE_INFINITY, "must be finite inches");
        assertInvalidOffsets(Double.NEGATIVE_INFINITY, "must be finite inches");
        assertInvalidOffsets(Double.MAX_VALUE, "Pinpoint millimetre float");
        assertInvalidOffsets(-Double.MAX_VALUE, "Pinpoint millimetre float");
        assertInvalidOffsets(Double.MIN_VALUE, "preserve a nonzero value");
        assertInvalidOffsets(-Double.MIN_VALUE, "preserve a nonzero value");

        PinpointOdometryPredictor.Config zero = PinpointOdometryPredictor.Config.defaults();
        zero.forwardPodOffsetLeftInches = -0.0;
        zero.strafePodOffsetForwardInches = 0.0;
        zero.validatedCopy("pinpoint");

        PinpointOdometryPredictor.Config signed = PinpointOdometryPredictor.Config.defaults();
        signed.forwardPodOffsetLeftInches = -6.25;
        signed.strafePodOffsetForwardInches = 4.75;
        PinpointOdometryPredictor.Config captured = signed.validatedCopy("pinpoint");
        assertEquals(-6.25, captured.forwardPodOffsetLeftInches, 0.0);
        assertEquals(4.75, captured.strafePodOffsetForwardInches, 0.0);
    }

    @Test
    public void encoderResolutionFactoriesAndActiveVendorTranslationAreExact() {
        RuntimeException nullPod = capture(() ->
                PinpointOdometryPredictor.EncoderResolution.forGoBildaPod(null));
        assertTrue(nullPod instanceof NullPointerException);
        assertTrue(nullPod.getMessage().contains("pod"));

        assertInvalidCustomFactory(0.0);
        assertInvalidCustomFactory(-1.0);
        assertInvalidCustomFactory(Double.NaN);
        assertInvalidCustomFactory(Double.POSITIVE_INFINITY);
        assertInvalidCustomFactory(Double.NEGATIVE_INFINITY);

        PinpointOdometryPredictor.Config underflow = PinpointOdometryPredictor.Config.defaults();
        underflow.encoderResolution =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(Double.MIN_VALUE);
        RuntimeException underflowFailure = capture(() -> underflow.validatedCopy("pinpoint"));
        assertTrue(underflowFailure.getMessage().contains("pinpoint.encoderResolution"));
        assertTrue(underflowFailure.getMessage().contains("finite positive Pinpoint float"));

        PinpointOdometryPredictor.Config overflow = PinpointOdometryPredictor.Config.defaults();
        overflow.encoderResolution =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(Double.MAX_VALUE);
        RuntimeException overflowFailure = capture(() -> overflow.validatedCopy("pinpoint"));
        assertTrue(overflowFailure.getMessage().contains("got " + Double.MAX_VALUE));

        PinpointOdometryPredictor.Config valid = PinpointOdometryPredictor.Config.defaults();
        valid.encoderResolution =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(1234.5);
        valid.validatedCopy("pinpoint");
    }

    @Test
    public void defaultsSelectExactlyTheGoBildaFourBarPod() {
        AtomicReference<GoBildaPinpointDriver.GoBildaOdometryPods> selectedPod =
                new AtomicReference<>();
        AtomicInteger podCalls = new AtomicInteger();
        AtomicInteger customCalls = new AtomicInteger();

        PinpointOdometryPredictor.Config.defaults().encoderResolution.applyTo(
                pod -> {
                    podCalls.incrementAndGet();
                    selectedPod.set(pod);
                },
                value -> customCalls.incrementAndGet()
        );

        assertEquals(1, podCalls.get());
        assertEquals(0, customCalls.get());
        assertEquals(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                selectedPod.get()
        );
    }

    @Test
    public void taggedResolutionRequiresBothReceiversAndInvokesExactlyOne() {
        AtomicReference<GoBildaPinpointDriver.GoBildaOdometryPods> selectedPod =
                new AtomicReference<>();
        AtomicReference<Double> selectedCustom = new AtomicReference<>();
        AtomicInteger podCalls = new AtomicInteger();
        AtomicInteger customCalls = new AtomicInteger();

        PinpointOdometryPredictor.EncoderResolution preset =
                PinpointOdometryPredictor.EncoderResolution.forGoBildaPod(
                        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                );
        preset.applyTo(
                pod -> {
                    podCalls.incrementAndGet();
                    selectedPod.set(pod);
                },
                value -> {
                    customCalls.incrementAndGet();
                    selectedCustom.set(value);
                }
        );
        assertEquals(1, podCalls.get());
        assertEquals(0, customCalls.get());
        assertEquals(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD,
                selectedPod.get()
        );
        assertNull(selectedCustom.get());
        assertTrue(preset.toString().contains("goBILDA_SWINGARM_POD"));

        PinpointOdometryPredictor.EncoderResolution custom =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(777.25);
        custom.applyTo(
                pod -> podCalls.incrementAndGet(),
                value -> {
                    customCalls.incrementAndGet();
                    selectedCustom.set(value);
                }
        );
        assertEquals(1, podCalls.get());
        assertEquals(1, customCalls.get());
        assertEquals(777.25, selectedCustom.get(), 0.0);
        assertTrue(custom.toString().contains("ticksPerInch=777.25"));

        AtomicInteger sideEffects = new AtomicInteger();
        RuntimeException missingCustom = capture(() -> preset.applyTo(
                ignored -> sideEffects.incrementAndGet(),
                null
        ));
        assertTrue(missingCustom instanceof NullPointerException);
        assertEquals(0, sideEffects.get());
        RuntimeException missingPod = capture(() -> custom.applyTo(
                null,
                ignored -> sideEffects.incrementAndGet()
        ));
        assertTrue(missingPod instanceof NullPointerException);
        assertEquals(0, sideEffects.get());
    }

    @Test
    public void directionsYawAndQualityUseTheirExactDomains() {
        PinpointOdometryPredictor.Config config = PinpointOdometryPredictor.Config.defaults();
        config.forwardPodDirection = null;
        assertFieldFailure(config, "forwardPodDirection", "must not be null");

        config = PinpointOdometryPredictor.Config.defaults();
        config.strafePodDirection = null;
        assertFieldFailure(config, "strafePodDirection", "must not be null");

        double[] invalidYaw = {
                0.0,
                -1.0,
                Double.MIN_VALUE,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.MAX_VALUE
        };
        for (double value : invalidYaw) {
            config = PinpointOdometryPredictor.Config.defaults();
            config.yawScalar = value;
            assertFieldFailure(config, "yawScalar", "finite and > 0");
        }
        config = PinpointOdometryPredictor.Config.defaults();
        config.yawScalar = null;
        config.validatedCopy("pinpoint");
        config.yawScalar = 1.001;
        config.validatedCopy("pinpoint");

        double[] invalidQuality = {
                -Double.MIN_VALUE,
                1.0 + Math.ulp(1.0),
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };
        for (double value : invalidQuality) {
            config = PinpointOdometryPredictor.Config.defaults();
            config.quality = value;
            assertFieldFailure(config, "quality", "[0, 1]");
        }
        config = PinpointOdometryPredictor.Config.defaults();
        config.quality = 0.0;
        config.validatedCopy("pinpoint");
        config.quality = 1.0;
        config.validatedCopy("pinpoint");
    }

    @Test
    public void nullEncoderChoiceFailsAtActiveCapture() {
        PinpointOdometryPredictor.Config config = PinpointOdometryPredictor.Config.defaults();
        config.encoderResolution = null;

        RuntimeException failure = capture(() -> config.validatedCopy("pinpoint"));

        assertTrue(failure instanceof NullPointerException);
        assertTrue(failure.getMessage().contains("pinpoint.encoderResolution must not be null"));
    }

    private static void assertInvalidOffsets(double value, String expectedText) {
        assertInvalidOffset(value, expectedText, true);
        assertInvalidOffset(value, expectedText, false);
    }

    private static void assertInvalidOffset(double value,
                                            String expectedText,
                                            boolean forwardPod) {
        PinpointOdometryPredictor.Config config = PinpointOdometryPredictor.Config.defaults();
        String field;
        if (forwardPod) {
            config.forwardPodOffsetLeftInches = value;
            field = "forwardPodOffsetLeftInches";
        } else {
            config.strafePodOffsetForwardInches = value;
            field = "strafePodOffsetForwardInches";
        }
        RuntimeException failure = capture(() -> config.validatedCopy("pinpoint"));
        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(failure.getMessage().contains("pinpoint." + field));
        assertTrue(failure.getMessage(), failure.getMessage().contains(expectedText));
        assertTrue(failure.getMessage().contains("got " + value));
    }

    private static void assertInvalidCustomFactory(double value) {
        RuntimeException failure = capture(() ->
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(value));
        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(failure.getMessage().contains("finite and > 0"));
        assertTrue(failure.getMessage().contains("got " + value));
    }

    private static void assertFieldFailure(PinpointOdometryPredictor.Config config,
                                           String field,
                                           String expectedText) {
        RuntimeException failure = capture(() -> config.validatedCopy("pinpoint"));
        assertTrue(failure.getMessage(), failure.getMessage().contains("pinpoint." + field));
        assertTrue(failure.getMessage(), failure.getMessage().contains(expectedText));
    }

    private static RuntimeException capture(Runnable action) {
        try {
            action.run();
            fail("Expected failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }
}
