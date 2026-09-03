package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the one-factory, one-Task-entry API and its side-effect-free staged construction. */
public final class ScalarTasksApiTest {

    @Test
    public void scalarTargetHasOneExactFrameworkFactoryAndResetsToItsInitialValue() {
        assertEquals(setOf(signature(ScalarTarget.class, "create", double.class)),
                publicStaticMethodSignatures(ScalarTarget.class));
        assertEquals(0, ScalarTarget.class.getDeclaredClasses().length);
        assertFalse(publicStaticMethodNames(ScalarSource.class).contains("mutable"));

        ScalarTarget target = ScalarTarget.create(2.5);
        target.set(-4.0);
        assertEquals(-4.0, target.get(), 0.0);
        target.reset();
        assertEquals(2.5, target.get(), 0.0);
    }

    @Test
    public void scalarTasksHasOnePublicFactoryAndExactLifetimeStages() throws Exception {
        assertEquals(setOf(signature(ScalarTasks.SetReadyStep.class, "set",
                        ScalarTarget.class, double.class)),
                publicStaticMethodSignatures(ScalarTasks.class));
        assertEquals(setOf(
                        signature(Task.class, "build"),
                        signature(ScalarTasks.TimedEndStep.class, "forSeconds", double.class),
                        signature(ScalarTasks.ReachedCancellationStep.class,
                                "untilReachedBy", Plant.class)),
                publicDeclaredMethodSignatures(ScalarTasks.SetReadyStep.class));
        assertEquals(setOf(
                        signature(ScalarTasks.TimedBuildStep.class, "leaveThere"),
                        signature(ScalarTasks.TimedBuildStep.class, "then", double.class)),
                publicDeclaredMethodSignatures(ScalarTasks.TimedEndStep.class));
        assertEquals(setOf(signature(Task.class, "build")),
                publicDeclaredMethodSignatures(ScalarTasks.TimedBuildStep.class));
        assertEquals(setOf(
                        signature(ScalarTasks.ReachedReadyStep.class, "cancelTo", double.class),
                        signature(ScalarTasks.ReachedReadyStep.class, "leaveRequestOnCancel")),
                publicDeclaredMethodSignatures(ScalarTasks.ReachedCancellationStep.class));
        assertEquals(setOf(
                        signature(Task.class, "build"),
                        signature(ScalarTasks.ReachedReadyStep.class, "stableFor", double.class),
                        signature(ScalarTasks.ReachedReadyStep.class, "timeout", double.class)),
                publicDeclaredMethodSignatures(ScalarTasks.ReachedReadyStep.class));
        assertFalse(publicStaticMethodNames(ScalarTasks.class).contains("move"));
        assertFalse(publicStaticMethodNames(ScalarTasks.class).contains("once"));
        assertFalse(publicStaticMethodNames(PlantTargets.class).contains("fromScalar"));
    }

    @Test
    public void factoryAndBuildDoNotReadOrWriteAndEachBuildCreatesAFreshTask() {
        CountingTarget target = new CountingTarget();
        ScalarTasks.SetReadyStep builder = ScalarTasks.set(target, 4.0);
        Task first = builder.build();
        Task second = builder.build();

        assertNotSame(first, second);
        assertEquals(0, target.getCount);
        assertEquals(0, target.setCount);

        ManualLoopClock clock = new ManualLoopClock();
        first.start(clock.clock());
        second.start(clock.clock());

        assertEquals(2, target.setCount);
        assertEquals(4.0, target.value, 0.0);
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, second.getOutcome());
    }

    @Test
    public void retainedTimedAndFeedbackTerminalStagesArePureReusableFactories() {
        CountingTarget target = new CountingTarget();

        ScalarTasks.TimedEndStep timedEnd = ScalarTasks.set(target, 4.0).forSeconds(0.25);
        ScalarTasks.TimedBuildStep timedLeave = timedEnd.leaveThere();
        ScalarTasks.TimedBuildStep timedThen = timedEnd.then(0.0);
        Task timedLeaveFirst = timedLeave.build();
        Task timedLeaveSecond = timedLeave.build();
        Task timedThenFirst = timedThen.build();
        Task timedThenSecond = timedThen.build();

        TestPlant plant = new TestPlant(target, true);
        ScalarTasks.ReachedCancellationStep cancellation = ScalarTasks.set(target, 8.0)
                .untilReachedBy(plant);
        ScalarTasks.ReachedReadyStep feedbackLeave = cancellation.leaveRequestOnCancel();
        ScalarTasks.ReachedReadyStep feedbackCancel = cancellation.cancelTo(-1.0);
        ScalarTasks.ReachedReadyStep configured = feedbackLeave
                .stableFor(0.1)
                .timeout(0.5);
        Task feedbackLeaveFirst = feedbackLeave.build();
        Task feedbackLeaveSecond = feedbackLeave.build();
        Task feedbackCancelFirst = feedbackCancel.build();
        Task feedbackCancelSecond = feedbackCancel.build();
        Task configuredFirst = configured.build();
        Task configuredSecond = configured.build();

        assertNotSame(timedLeaveFirst, timedLeaveSecond);
        assertNotSame(timedThenFirst, timedThenSecond);
        assertNotSame(timedLeaveFirst, timedThenFirst);
        assertNotSame(feedbackLeaveFirst, feedbackLeaveSecond);
        assertNotSame(feedbackCancelFirst, feedbackCancelSecond);
        assertNotSame(feedbackLeaveFirst, feedbackCancelFirst);
        assertNotSame(configuredFirst, configuredSecond);
        assertEquals(0, target.getCount);
        assertEquals(0, target.setCount);
    }

    @Test
    public void untilReachedByRequiresTheSameCommandTargetAndFeedback() {
        CountingTarget requested = new CountingTarget();
        CountingTarget different = new CountingTarget();

        assertFailure(
                () -> ScalarTasks.set(requested, 1.0)
                        .untilReachedBy(new TestPlant(null, true)),
                IllegalStateException.class,
                "target graph");
        assertFailure(
                () -> ScalarTasks.set(requested, 1.0)
                        .untilReachedBy(new TestPlant(different, true)),
                IllegalArgumentException.class,
                "different objects");
        assertFailure(
                () -> ScalarTasks.set(requested, 1.0)
                        .untilReachedBy(new TestPlant(requested, false)),
                IllegalStateException.class,
                "feedback");

        assertEquals(0, requested.getCount);
        assertEquals(0, requested.setCount);
    }

    @Test
    public void customPlantFallbackDoesNotCompleteAfterSharedCommandChanges() {
        ScalarTarget shared = ScalarTarget.create(0.0);
        TestPlant plant = new TestPlant(shared, true);
        plant.atTarget = true;
        Task move = ScalarTasks.set(shared, 1.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        move.start(clock.clock());
        shared.set(2.0);
        move.update(clock.clock());

        assertFalse(move.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, move.getOutcome());
        move.cancel();
    }

    @Test
    public void immutableFeedbackStagesRejectRepeatedOptionalAnswersWithoutPoisoningBase() {
        CountingTarget target = new CountingTarget();
        TestPlant plant = new TestPlant(target, true);
        ScalarTasks.ReachedReadyStep base = ScalarTasks.set(target, 3.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel();

        ScalarTasks.ReachedReadyStep stable = base.stableFor(0.1);
        assertFailure(() -> stable.stableFor(0.2), IllegalStateException.class,
                "already been answered");
        ScalarTasks.ReachedReadyStep timed = base.timeout(0.5);
        assertFailure(() -> timed.timeout(1.0), IllegalStateException.class,
                "already been answered");

        assertNotSame(stable.build(), timed.build());
        assertEquals(0, target.getCount);
        assertEquals(0, target.setCount);
    }

    @Test
    public void everyNumericBuilderAnswerFailsFastWithoutWriting() {
        CountingTarget target = new CountingTarget();
        TestPlant plant = new TestPlant(target, true);

        assertFailure(() -> ScalarTasks.set(target, Double.NaN),
                IllegalArgumentException.class, "finite");
        assertFailure(() -> ScalarTasks.set(target, 1.0).forSeconds(-0.1),
                IllegalArgumentException.class, ">= 0");
        assertFailure(() -> ScalarTasks.set(target, 1.0).forSeconds(Double.POSITIVE_INFINITY),
                IllegalArgumentException.class, "finite");
        assertFailure(() -> ScalarTasks.set(target, 1.0).forSeconds(0.1).then(Double.NaN),
                IllegalArgumentException.class, "finite");

        ScalarTasks.ReachedReadyStep reached = ScalarTasks.set(target, 1.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel();
        assertFailure(() -> reached.stableFor(-0.1),
                IllegalArgumentException.class, ">= 0");
        assertFailure(() -> reached.timeout(0.0),
                IllegalArgumentException.class, "> 0");

        assertEquals(0, target.getCount);
        assertEquals(0, target.setCount);
    }

    private static Set<String> publicStaticMethodNames(Class<?> type) {
        Set<String> names = new HashSet<String>();
        for (Method method : type.getDeclaredMethods()) {
            int modifiers = method.getModifiers();
            if (Modifier.isPublic(modifiers) && Modifier.isStatic(modifiers)) {
                names.add(method.getName());
            }
        }
        return names;
    }

    private static Set<String> publicStaticMethodSignatures(Class<?> type) {
        Set<String> signatures = new HashSet<String>();
        for (Method method : type.getDeclaredMethods()) {
            int modifiers = method.getModifiers();
            if (Modifier.isPublic(modifiers) && Modifier.isStatic(modifiers)) {
                signatures.add(signature(method));
            }
        }
        return signatures;
    }

    private static Set<String> publicDeclaredMethodSignatures(Class<?> type) {
        Set<String> signatures = new HashSet<String>();
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) signatures.add(signature(method));
        }
        return signatures;
    }

    private static String signature(Method method) {
        return signature(method.getReturnType(), method.getName(), method.getParameterTypes());
    }

    private static String signature(Class<?> returnType, String name, Class<?>... parameterTypes) {
        StringBuilder result = new StringBuilder(returnType.getName())
                .append(' ')
                .append(name)
                .append('(');
        for (int i = 0; i < parameterTypes.length; i++) {
            if (i > 0) result.append(',');
            result.append(parameterTypes[i].getName());
        }
        return result.append(')').toString();
    }

    private static Set<String> setOf(String... entries) {
        Set<String> values = new HashSet<String>();
        for (String entry : entries) values.add(entry);
        return values;
    }

    private static void assertFailure(Runnable action,
                                      Class<? extends RuntimeException> expectedType,
                                      String messagePart) {
        try {
            action.run();
            fail("expected " + expectedType.getSimpleName());
        } catch (RuntimeException expected) {
            assertTrue("wrong exception: " + expected,
                    expectedType.isInstance(expected));
            assertTrue(expected.getMessage().contains(messagePart));
        }
    }

    private static final class CountingTarget implements ScalarTarget {
        private int setCount;
        private int getCount;
        private double value;

        @Override
        public void set(double value) {
            setCount++;
            this.value = value;
        }

        @Override
        public double get() {
            getCount++;
            return value;
        }
    }

    private static final class TestPlant implements Plant {
        private final ScalarTarget commandTarget;
        private final boolean feedback;
        private boolean atTarget;

        private TestPlant(ScalarTarget commandTarget, boolean feedback) {
            this.commandTarget = commandTarget;
            this.feedback = feedback;
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public double getRequestedTarget() {
            return 0.0;
        }

        @Override
        public double getAppliedTarget() {
            return 0.0;
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasCommandTarget() {
            return commandTarget != null;
        }

        @Override
        public ScalarTarget commandTarget() {
            if (commandTarget == null) return Plant.super.commandTarget();
            return commandTarget;
        }

        @Override
        public boolean hasFeedback() {
            return feedback;
        }

        @Override
        public boolean atTarget(double target) {
            return atTarget;
        }

        @Override
        public void stop() {
        }
    }
}
