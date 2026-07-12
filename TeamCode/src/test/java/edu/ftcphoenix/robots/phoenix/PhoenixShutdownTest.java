package edu.ftcphoenix.robots.phoenix;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix's internal exact-once and best-effort shutdown rules. */
public final class PhoenixShutdownTest {

    @Test
    public void actionsRunOnceInDeclaredOrderAndSkipMissingOwners() {
        PhoenixShutdown shutdown = new PhoenixShutdown();
        List<String> actions = new ArrayList<>();

        shutdown.beginInitialization("initTeleOp");
        shutdown.run(
                () -> actions.add("cancel behavior"),
                null,
                () -> actions.add("stop scoring"),
                () -> actions.add("stop auto drive"),
                () -> actions.add("close vision")
        );
        shutdown.run(() -> actions.add("unexpected second stop"));

        assertEquals(
                Arrays.asList(
                        "cancel behavior",
                        "stop scoring",
                        "stop auto drive",
                        "close vision"
                ),
                actions
        );
    }

    @Test
    public void cleanupContinuesAfterFailureAndAggregatesLaterFailures() {
        PhoenixShutdown shutdown = new PhoenixShutdown();
        List<String> actions = new ArrayList<>();
        RuntimeException firstFailure = new IllegalStateException("first");
        RuntimeException laterFailure = new IllegalArgumentException("later");

        try {
            shutdown.run(
                    () -> {
                        actions.add("first");
                        throw firstFailure;
                    },
                    () -> actions.add("middle"),
                    () -> {
                        actions.add("last");
                        throw laterFailure;
                    }
            );
            fail("Expected the first cleanup failure");
        } catch (RuntimeException actual) {
            assertSame(firstFailure, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertSame(laterFailure, actual.getSuppressed()[0]);
        }

        assertEquals(Arrays.asList("first", "middle", "last"), actions);
    }

    @Test
    public void initializationIsRejectedAfterStop() {
        PhoenixShutdown shutdown = new PhoenixShutdown();
        shutdown.run();

        try {
            shutdown.ensureOpen("initAuto");
            fail("Expected initialization after stop to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("create a new PhoenixRobot"));
        }
    }

    @Test
    public void repeatedOrCrossModeInitializationCannotOverwriteTheActiveGraph() {
        PhoenixShutdown shutdown = new PhoenixShutdown();
        List<String> actions = new ArrayList<>();
        shutdown.beginInitialization("initAuto");

        try {
            shutdown.beginInitialization("initTeleOp");
            fail("Expected a second mode initialization to be rejected");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("already initialized"));
            assertTrue(expected.getMessage().contains("create a new PhoenixRobot"));
        }

        shutdown.run(() -> actions.add("stop original graph"));
        assertEquals(Arrays.asList("stop original graph"), actions);
    }
}
