package edu.ftcphoenix.fw.drive.route;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks down the factory-only, explicit-timeout construction API for route Tasks. */
public final class RouteTasksApiTest {

    @Test
    public void routeTasksExposesExactlyFourNamedFactoriesReturningRouteTask() {
        Set<String> actualSignatures = new TreeSet<>();

        for (Method method : RouteTasks.class.getDeclaredMethods()) {
            if (!Modifier.isPublic(method.getModifiers()) || method.isSynthetic()) {
                continue;
            }
            assertTrue(Modifier.isStatic(method.getModifiers()));
            assertEquals(RouteTask.class, method.getReturnType());
            actualSignatures.add(signature(method));
        }

        Set<String> expectedSignatures = new TreeSet<>(Arrays.asList(
                "follow(String,RouteFollower,Object,double)",
                "followBuiltAtStart(String,RouteFollower,Supplier,double)",
                "followBuiltAtStartWithoutTaskTimeout(String,RouteFollower,Supplier)",
                "followWithoutTaskTimeout(String,RouteFollower,Object)"
        ));
        assertEquals(expectedSignatures, actualSignatures);
    }

    @Test
    public void routeTaskHasNoPublicConstructorOrLegacyConfigType() {
        assertEquals(0, RouteTask.class.getConstructors().length);

        for (Class<?> nestedType : RouteTask.class.getDeclaredClasses()) {
            assertFalse("RouteTask.Config must be removed", "Config".equals(nestedType.getSimpleName()));
        }
    }

    @Test
    public void everyFactoryRejectsMissingOrBlankDebugNameBeforeSideEffects() {
        String[] invalidNames = {null, "", " \t "};

        for (String invalidName : invalidNames) {
            RecordingFollower follower = new RecordingFollower();
            AtomicInteger supplierCalls = new AtomicInteger();

            assertInvalidName(() ->
                    RouteTasks.follow(invalidName, follower, "eager-route", 1.0));
            assertInvalidName(() ->
                    RouteTasks.followWithoutTaskTimeout(invalidName, follower, "eager-route"));
            assertInvalidName(() ->
                    RouteTasks.followBuiltAtStart(
                            invalidName,
                            follower,
                            countedRoute(supplierCalls),
                            1.0));
            assertInvalidName(() ->
                    RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                            invalidName,
                            follower,
                            countedRoute(supplierCalls)));

            assertEquals(0, supplierCalls.get());
            assertEquals(0, follower.followCount);
        }
    }

    @Test
    public void timedFactoriesRejectNonFiniteOrNonPositiveTimeoutBeforeSideEffects() {
        double[] invalidTimeouts = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                -1.0,
                -0.0,
                0.0,
                Double.POSITIVE_INFINITY
        };

        for (double invalidTimeout : invalidTimeouts) {
            RecordingFollower follower = new RecordingFollower();
            AtomicInteger supplierCalls = new AtomicInteger();

            assertInvalidTimeout(
                    invalidTimeout,
                    "followWithoutTaskTimeout",
                    () -> RouteTasks.follow(
                            "invalidEagerTimeout",
                            follower,
                            "eager-route",
                            invalidTimeout));
            assertInvalidTimeout(
                    invalidTimeout,
                    "followBuiltAtStartWithoutTaskTimeout",
                    () -> RouteTasks.followBuiltAtStart(
                            "invalidStartTimeTimeout",
                            follower,
                            countedRoute(supplierCalls),
                            invalidTimeout));

            assertEquals(0, supplierCalls.get());
            assertEquals(0, follower.followCount);
        }
    }

    @Test
    public void eagerWithoutTaskTimeoutRemainsActiveAfterLongElapsedTime() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("unboundedEager", follower, "eager-route");

        task.start(manualClock.clock());
        manualClock.nextCycle(1_000.0);
        task.update(manualClock.clock());

        assertFalse(task.isComplete());
        assertEquals(RouteStatus.ACTIVE, task.getRouteStatus());
        assertEquals(0, follower.current.cancelCount);
    }

    @Test
    public void startTimeWithoutTaskTimeoutBuildsOnceAndRemainsActiveAfterLongElapsedTime() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        AtomicInteger supplierCalls = new AtomicInteger();
        RouteTask<String> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "unboundedStartTime",
                follower,
                countedRoute(supplierCalls));

        assertEquals(0, supplierCalls.get());
        task.start(manualClock.clock());
        manualClock.nextCycle(1_000.0);
        task.update(manualClock.clock());

        assertEquals(1, supplierCalls.get());
        assertFalse(task.isComplete());
        assertEquals(RouteStatus.ACTIVE, task.getRouteStatus());
        assertEquals(0, follower.current.cancelCount);
    }

    private static Supplier<String> countedRoute(AtomicInteger calls) {
        return () -> {
            calls.incrementAndGet();
            return "start-time-route";
        };
    }

    private static String signature(Method method) {
        StringBuilder text = new StringBuilder(method.getName()).append('(');
        Class<?>[] parameterTypes = method.getParameterTypes();
        for (int index = 0; index < parameterTypes.length; index++) {
            if (index > 0) {
                text.append(',');
            }
            text.append(parameterTypes[index].getSimpleName());
        }
        return text.append(')').toString();
    }

    private static void assertInvalidName(ThrowingAction action) {
        try {
            action.run();
            fail("expected missing or blank debugName to fail");
        } catch (IllegalArgumentException expected) {
            String message = String.valueOf(expected.getMessage()).toLowerCase();
            assertTrue(message, message.contains("debugname") || message.contains("debug name"));
            assertTrue(message, message.contains("blank"));
        }
    }

    private static void assertInvalidTimeout(double invalidTimeout,
                                             String expectedNoTimeoutFactory,
                                             ThrowingAction action) {
        try {
            action.run();
            fail("expected taskTimeoutSec=" + invalidTimeout + " to fail");
        } catch (IllegalArgumentException expected) {
            String message = String.valueOf(expected.getMessage());
            assertTrue(message, message.contains("taskTimeoutSec"));
            assertTrue(message, message.contains(String.valueOf(invalidTimeout)));
            assertTrue(message, message.contains("> 0"));
            assertTrue(message, message.toLowerCase().contains("finite"));
            assertTrue(message, message.contains(expectedNoTimeoutFactory));
        }
    }

    private interface ThrowingAction {
        void run();
    }

    private static final class RecordingFollower implements RouteFollower<String> {
        private int followCount;
        private RecordingExecution current;

        @Override
        public RouteExecution follow(String route) {
            followCount++;
            current = new RecordingExecution();
            return current;
        }

        @Override
        public void update(LoopClock clock) {
            // Keep the current execution active.
        }
    }

    private static final class RecordingExecution extends RouteExecution {
        private RouteStatus integrationStatus = RouteStatus.ACTIVE;
        private int cancelCount;

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            integrationStatus = RouteStatus.CANCELLED;
            cancelCount++;
        }
    }
}
