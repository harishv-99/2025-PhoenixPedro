package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.ftcphoenix.fw.haptic.HapticSink;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the small haptic contract and its best-effort FTC gamepad boundary. */
public final class FtcHapticsTest {

    @Test
    public void publicApiHasOneCoreOperationShapeAndOneFtcFactory() {
        List<String> sinkMethods = new ArrayList<>();
        for (Method method : HapticSink.class.getDeclaredMethods()) {
            sinkMethods.add(methodSignature(method));
        }
        Collections.sort(sinkMethods);
        assertEquals(Arrays.asList(
                "pulse(double,double):void",
                "stop():void"
        ), sinkMethods);

        for (Constructor<?> constructor : FtcHaptics.class.getDeclaredConstructors()) {
            assertTrue(Modifier.isPrivate(constructor.getModifiers()));
        }

        List<String> publicFactories = new ArrayList<>();
        for (Method method : FtcHaptics.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())
                    && Modifier.isStatic(method.getModifiers())) {
                publicFactories.add(methodSignature(method));
            }
        }
        assertEquals(
                Collections.singletonList("gamepad(Gamepad):HapticSink"),
                publicFactories);

        HapticSink sink = FtcHaptics.gamepad(new Gamepad());
        assertSame(FtcHaptics.class, sink.getClass().getEnclosingClass());
        assertTrue(Modifier.isPrivate(sink.getClass().getModifiers()));
    }

    @Test
    public void gamepadFactoryRejectsNull() {
        assertThrowsContains(
                IllegalArgumentException.class,
                () -> FtcHaptics.gamepad(null),
                "gamepad");
    }

    @Test
    public void pulseMapsStrengthToBothChannelsAndRoundsDurationUp() {
        Gamepad gamepad = new Gamepad();
        HapticSink sink = FtcHaptics.gamepad(gamepad);

        sink.pulse(0.5, 0.123_001);

        Gamepad.RumbleEffect effect = gamepad.rumbleQueue.poll();
        assertOneStep(effect, 128, 128, 124);
        assertNull(gamepad.rumbleQueue.poll());
    }

    @Test
    public void everyPositiveSubMillisecondPulseRemainsObservableToSdkQueue() {
        Gamepad gamepad = new Gamepad();
        HapticSink sink = FtcHaptics.gamepad(gamepad);

        sink.pulse(1.0, Double.MIN_VALUE);

        assertOneStep(gamepad.rumbleQueue.poll(), 255, 255, 1);
    }

    @Test
    public void everyPositiveStrengthRemainsNonzeroInSdkQueue() {
        Gamepad gamepad = new Gamepad();
        HapticSink sink = FtcHaptics.gamepad(gamepad);

        sink.pulse(Double.MIN_VALUE, 0.1);

        assertOneStep(gamepad.rumbleQueue.poll(), 1, 1, 100);
    }

    @Test
    public void maximumRepresentableDurationIsAcceptedAndLongerDurationIsRejected() {
        Gamepad acceptedGamepad = new Gamepad();
        HapticSink accepted = FtcHaptics.gamepad(acceptedGamepad);

        accepted.pulse(1.0, Integer.MAX_VALUE / 1000.0);

        assertOneStep(
                acceptedGamepad.rumbleQueue.poll(),
                255,
                255,
                Integer.MAX_VALUE);

        Gamepad rejectedGamepad = new Gamepad();
        HapticSink rejected = FtcHaptics.gamepad(rejectedGamepad);
        double tooLongSec = Math.nextUp(Integer.MAX_VALUE / 1000.0);

        assertThrowsContains(
                IllegalArgumentException.class,
                () -> rejected.pulse(1.0, tooLongSec),
                "durationSec");
        assertNull(rejectedGamepad.rumbleQueue.poll());
    }

    @Test
    public void invalidStrengthsFailBeforeQueueingAnEffect() {
        double[] invalidStrengths = {
                0.0,
                -0.01,
                1.01,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };

        for (double invalidStrength : invalidStrengths) {
            Gamepad gamepad = new Gamepad();
            HapticSink sink = FtcHaptics.gamepad(gamepad);

            assertThrowsContains(
                    IllegalArgumentException.class,
                    () -> sink.pulse(invalidStrength, 0.1),
                    "strength");
            assertNull("strength " + invalidStrength, gamepad.rumbleQueue.poll());
        }
    }

    @Test
    public void invalidDurationsFailBeforeQueueingAnEffect() {
        double[] invalidDurations = {
                0.0,
                -0.01,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };

        for (double invalidDuration : invalidDurations) {
            Gamepad gamepad = new Gamepad();
            HapticSink sink = FtcHaptics.gamepad(gamepad);

            assertThrowsContains(
                    IllegalArgumentException.class,
                    () -> sink.pulse(0.5, invalidDuration),
                    "durationSec");
            assertNull("duration " + invalidDuration, gamepad.rumbleQueue.poll());
        }
    }

    @Test
    public void latestPulseReplacesAnUnsentPulseInTheSdkQueue() {
        Gamepad gamepad = new Gamepad();
        HapticSink sink = FtcHaptics.gamepad(gamepad);

        sink.pulse(0.2, 0.1);
        sink.pulse(0.8, 0.4);

        assertOneStep(gamepad.rumbleQueue.poll(), 204, 204, 400);
        assertNull(gamepad.rumbleQueue.poll());
    }

    @Test
    public void eachSinkQueuesEffectsOnlyForItsOwnGamepad() {
        Gamepad driverGamepad = new Gamepad();
        Gamepad operatorGamepad = new Gamepad();
        HapticSink driver = FtcHaptics.gamepad(driverGamepad);
        HapticSink operator = FtcHaptics.gamepad(operatorGamepad);

        driver.pulse(0.25, 0.1);
        operator.pulse(0.75, 0.2);

        assertOneStep(driverGamepad.rumbleQueue.poll(), 64, 64, 100);
        assertOneStep(operatorGamepad.rumbleQueue.poll(), 191, 191, 200);
        assertNull(driverGamepad.rumbleQueue.poll());
        assertNull(operatorGamepad.rumbleQueue.poll());
    }

    @Test
    public void stopQueuesBestEffortZeroRequestAndIsSafeToRepeat() {
        Gamepad gamepad = new Gamepad();
        HapticSink sink = FtcHaptics.gamepad(gamepad);

        sink.pulse(1.0, 1.0);
        sink.stop();

        assertOneStep(
                gamepad.rumbleQueue.poll(),
                0,
                0,
                Gamepad.RUMBLE_DURATION_CONTINUOUS);
        assertNull(gamepad.rumbleQueue.poll());

        sink.stop();
        assertOneStep(
                gamepad.rumbleQueue.poll(),
                0,
                0,
                Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    private static void assertOneStep(Gamepad.RumbleEffect effect,
                                      int expectedLarge,
                                      int expectedSmall,
                                      int expectedDurationMs) {
        assertTrue("expected one queued rumble effect", effect != null);
        assertEquals(1, effect.steps.size());
        Gamepad.RumbleEffect.Step step = effect.steps.get(0);
        assertEquals(expectedLarge, step.large);
        assertEquals(expectedSmall, step.small);
        assertEquals(expectedDurationMs, step.duration);
    }

    private static String methodSignature(Method method) {
        StringBuilder result = new StringBuilder(method.getName()).append('(');
        Class<?>[] parameterTypes = method.getParameterTypes();
        for (int i = 0; i < parameterTypes.length; i++) {
            if (i > 0) {
                result.append(',');
            }
            result.append(parameterTypes[i].getSimpleName());
        }
        return result.append("):")
                .append(method.getReturnType().getSimpleName())
                .toString();
    }

    private static void assertThrowsContains(Class<? extends Throwable> expectedType,
                                             ThrowingRunnable action,
                                             String expectedMessagePart) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (Throwable failure) {
            assertTrue(
                    "Expected " + expectedType.getSimpleName() + " but got " + failure,
                    expectedType.isInstance(failure));
            assertTrue(
                    "Expected message containing '" + expectedMessagePart + "' but got: "
                            + failure.getMessage(),
                    failure.getMessage() != null
                            && failure.getMessage().contains(expectedMessagePart));
        }
    }

    private interface ThrowingRunnable {
        void run() throws Throwable;
    }
}
