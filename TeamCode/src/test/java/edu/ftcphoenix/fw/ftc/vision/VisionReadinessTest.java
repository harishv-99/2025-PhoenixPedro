package edu.ftcphoenix.fw.ftc.vision;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused value-contract coverage for component readiness. */
public final class VisionReadinessTest {

    @Test
    public void readyIsSharedAndIndependentOfTargetVisibility() {
        VisionReadiness first = VisionReadiness.ready();
        VisionReadiness second = VisionReadiness.ready();

        assertSame(first, second);
        assertTrue(first.isReady());
        assertEquals("ready", first.reason());
        assertEquals("READY", first.toString());
    }

    @Test
    public void notReadyTrimsAndBehavesAsAValue() {
        VisionReadiness first = VisionReadiness.notReady("  camera opening  ");
        VisionReadiness second = VisionReadiness.notReady("camera opening");

        assertFalse(first.isReady());
        assertEquals("camera opening", first.reason());
        assertEquals(first, second);
        assertEquals(first.hashCode(), second.hashCode());
        assertEquals("NOT_READY(camera opening)", first.toString());
    }

    @Test
    public void notReadyRequiresAnActionableReason() {
        expectFailure(NullPointerException.class, () -> VisionReadiness.notReady(null));
        expectFailure(IllegalArgumentException.class, () -> VisionReadiness.notReady(""));
        expectFailure(IllegalArgumentException.class, () -> VisionReadiness.notReady("   "));
    }

    private static void expectFailure(
            Class<? extends RuntimeException> expectedType,
            Runnable action
    ) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (RuntimeException actual) {
            assertTrue("Unexpected failure: " + actual, expectedType.isInstance(actual));
        }
    }
}
