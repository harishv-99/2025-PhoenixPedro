package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the semantic scalar Task grammar and its side-effect-free staged construction. */
public final class SemanticScalarTasksApiTest {

    private enum Mode {
        IDLE,
        ACTIVE,
        INVALID,
        THROWING
    }

    @Test
    public void semanticScalarTasksHasOnePublicFactoryAndParallelLifetimeStages() {
        assertEquals(setOf(signature(SemanticScalarTasks.SetReadyStep.class, "set",
                        SemanticScalarCommand.class, Object.class)),
                publicStaticMethodSignatures(SemanticScalarTasks.class));
        assertEquals(setOf(
                        signature(Task.class, "build"),
                        signature(SemanticScalarTasks.TimedEndStep.class,
                                "forSeconds", double.class),
                        signature(SemanticScalarTasks.ReachedCancellationStep.class,
                                "untilReachedBy", Plant.class)),
                publicDeclaredMethodSignatures(SemanticScalarTasks.SetReadyStep.class));
        assertEquals(setOf(
                        signature(SemanticScalarTasks.TimedBuildStep.class, "leaveThere"),
                        signature(SemanticScalarTasks.TimedBuildStep.class,
                                "then", Object.class)),
                publicDeclaredMethodSignatures(SemanticScalarTasks.TimedEndStep.class));
        assertEquals(setOf(signature(Task.class, "build")),
                publicDeclaredMethodSignatures(SemanticScalarTasks.TimedBuildStep.class));
        assertEquals(setOf(
                        signature(SemanticScalarTasks.ReachedReadyStep.class,
                                "cancelTo", Object.class),
                        signature(SemanticScalarTasks.ReachedReadyStep.class,
                                "leaveRequestOnCancel")),
                publicDeclaredMethodSignatures(
                        SemanticScalarTasks.ReachedCancellationStep.class));
        assertEquals(setOf(
                        signature(Task.class, "build"),
                        signature(SemanticScalarTasks.ReachedReadyStep.class,
                                "stableFor", double.class),
                        signature(SemanticScalarTasks.ReachedReadyStep.class,
                                "timeout", double.class)),
                publicDeclaredMethodSignatures(SemanticScalarTasks.ReachedReadyStep.class));

        Set<String> allNames = new HashSet<String>();
        allNames.addAll(publicStaticMethodNames(SemanticScalarTasks.class));
        allNames.addAll(publicDeclaredMethodNames(SemanticScalarTasks.SetReadyStep.class));
        allNames.addAll(publicDeclaredMethodNames(SemanticScalarTasks.TimedEndStep.class));
        allNames.addAll(publicDeclaredMethodNames(
                SemanticScalarTasks.ReachedCancellationStep.class));
        allNames.addAll(publicDeclaredMethodNames(SemanticScalarTasks.ReachedReadyStep.class));
        assertFalse(allNames.contains("move"));
        assertFalse(allNames.contains("once"));
        assertFalse(allNames.contains("leaveTargetOnCancel"));
        assertFalse(allNames.contains("thenTarget"));
        assertFalse(allNames.contains("thenRequest"));
    }

    @Test
    public void setMapsAtTheBuilderBoundaryButPublishesOnlyFreshStartedOccurrences() {
        int[] mappings = {0};
        SemanticScalarCommand<Mode> command = SemanticScalarCommand.create(Mode.IDLE, mode -> {
            mappings[0]++;
            return mode == Mode.ACTIVE ? 0.75 : 0.0;
        });
        SemanticScalarCommand.Request<Mode> initial = command.request();

        SemanticScalarTasks.SetReadyStep<Mode> builder =
                SemanticScalarTasks.set(command, Mode.ACTIVE);
        Task first = builder.build();
        Task second = builder.build();

        assertEquals(2, mappings[0]);
        assertSame(initial, command.request());
        assertNotSame(first, second);

        ManualLoopClock time = new ManualLoopClock();
        first.start(time.clock());
        SemanticScalarCommand.Request<Mode> firstRequest = command.request();
        second.start(time.clock());
        SemanticScalarCommand.Request<Mode> secondRequest = command.request();

        assertNotSame(initial, firstRequest);
        assertNotSame(firstRequest, secondRequest);
        assertEquals(Mode.ACTIVE, firstRequest.semantic());
        assertEquals(0.75, firstRequest.commandTarget(), 0.0);
        assertEquals(Mode.ACTIVE, secondRequest.semantic());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, second.getOutcome());
        assertEquals("Starting prepared Tasks must not remap their values", 2, mappings[0]);
    }

    @Test
    public void retainedTerminalStagesArePureReusableTaskFactories() {
        int[] mappings = {0};
        SemanticScalarCommand<Mode> command = SemanticScalarCommand.create(Mode.IDLE, mode -> {
            mappings[0]++;
            return mode == Mode.ACTIVE ? 0.75 : 0.0;
        });
        SemanticScalarCommand.Request<Mode> initial = command.request();
        SemanticScalarTasks.SetReadyStep<Mode> set =
                SemanticScalarTasks.set(command, Mode.ACTIVE);

        SemanticScalarTasks.TimedEndStep<Mode> timedEnd = set.forSeconds(0.25);
        SemanticScalarTasks.TimedBuildStep timedLeave = timedEnd.leaveThere();
        SemanticScalarTasks.TimedBuildStep timedThen = timedEnd.then(Mode.IDLE);
        Task timedLeaveFirst = timedLeave.build();
        Task timedLeaveSecond = timedLeave.build();
        Task timedThenFirst = timedThen.build();
        Task timedThenSecond = timedThen.build();

        TestPlant plant = new TestPlant(command, true);
        SemanticScalarTasks.ReachedCancellationStep<Mode> cancellation =
                set.untilReachedBy(plant);
        SemanticScalarTasks.ReachedReadyStep<Mode> feedbackLeave =
                cancellation.leaveRequestOnCancel();
        SemanticScalarTasks.ReachedReadyStep<Mode> feedbackCancel =
                cancellation.cancelTo(Mode.IDLE);
        SemanticScalarTasks.ReachedReadyStep<Mode> configured = feedbackLeave
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
        assertSame(initial, command.request());
        assertEquals("initial, active, timed ending, and cancellation mappings", 4,
                mappings[0]);
    }

    @Test
    public void reusedTimedAndFeedbackBuildersPublishFreshStartOccurrences() {
        SemanticScalarCommand<Mode> command = command();
        ManualLoopClock time = new ManualLoopClock();

        SemanticScalarTasks.TimedBuildStep timedBuilder =
                SemanticScalarTasks.set(command, Mode.ACTIVE)
                        .forSeconds(1.0)
                        .leaveThere();
        Task firstTimed = timedBuilder.build();
        Task secondTimed = timedBuilder.build();
        firstTimed.start(time.clock());
        SemanticScalarCommand.Request<Mode> firstTimedRequest = command.request();
        firstTimed.cancel();
        secondTimed.start(time.clock());
        SemanticScalarCommand.Request<Mode> secondTimedRequest = command.request();
        secondTimed.cancel();

        SemanticScalarTasks.ReachedReadyStep<Mode> feedbackBuilder =
                SemanticScalarTasks.set(command, Mode.ACTIVE)
                        .untilReachedBy(new TestPlant(command, true))
                        .leaveRequestOnCancel();
        Task firstFeedback = feedbackBuilder.build();
        Task secondFeedback = feedbackBuilder.build();
        firstFeedback.start(time.clock());
        SemanticScalarCommand.Request<Mode> firstFeedbackRequest = command.request();
        firstFeedback.cancel();
        secondFeedback.start(time.clock());
        SemanticScalarCommand.Request<Mode> secondFeedbackRequest = command.request();
        secondFeedback.cancel();

        assertNotSame(firstTimedRequest, secondTimedRequest);
        assertNotSame(secondTimedRequest, firstFeedbackRequest);
        assertNotSame(firstFeedbackRequest, secondFeedbackRequest);
    }

    @Test
    public void untilReachedByRequiresTheExactSemanticCommandAndFeedback() {
        SemanticScalarCommand<Mode> requested = command();
        SemanticScalarCommand<Mode> different = command();
        SemanticScalarCommand.Request<Mode> initial = requested.request();
        SemanticScalarTasks.SetReadyStep<Mode> set =
                SemanticScalarTasks.set(requested, Mode.ACTIVE);

        assertFailure(() -> set.untilReachedBy(new TestPlant(null, true)),
                IllegalArgumentException.class, "targetExactlyFrom(command)");
        assertFailure(() -> set.untilReachedBy(new TestPlant(different, true)),
                IllegalArgumentException.class, "carries this exact");
        assertFailure(() -> set.untilReachedBy(new TestPlant(requested, false)),
                IllegalStateException.class, "feedback");

        SemanticScalarTasks.ReachedCancellationStep<Mode> accepted =
                set.untilReachedBy(new TestPlant(requested, true));
        assertTrue(accepted != null);
        assertSame(initial, requested.request());
    }

    @Test
    public void invalidMappingsAndBuilderAnswersFailWithoutPublishingOrPoisoningBaseStages() {
        RuntimeException mapperFailure = new RuntimeException("mapper failed");
        SemanticScalarCommand<Mode> command = SemanticScalarCommand.create(Mode.IDLE, mode -> {
            if (mode == Mode.INVALID) return Double.NaN;
            if (mode == Mode.THROWING) throw mapperFailure;
            return mode == Mode.ACTIVE ? 0.75 : 0.0;
        });
        SemanticScalarCommand.Request<Mode> initial = command.request();

        assertFailure(() -> SemanticScalarTasks.set(null, Mode.ACTIVE),
                NullPointerException.class, "command");
        assertFailure(() -> SemanticScalarTasks.set(command, null),
                NullPointerException.class, "semantic");
        assertFailure(() -> SemanticScalarTasks.set(command, Mode.INVALID),
                IllegalArgumentException.class, "finite target");
        try {
            SemanticScalarTasks.set(command, Mode.THROWING);
            fail("expected mapper failure");
        } catch (RuntimeException actual) {
            assertSame(mapperFailure, actual);
        }

        SemanticScalarTasks.SetReadyStep<Mode> set =
                SemanticScalarTasks.set(command, Mode.ACTIVE);
        assertFailure(() -> set.forSeconds(-0.1),
                IllegalArgumentException.class, ">= 0");
        assertFailure(() -> set.forSeconds(Double.POSITIVE_INFINITY),
                IllegalArgumentException.class, "finite");
        SemanticScalarTasks.TimedEndStep<Mode> timed = set.forSeconds(0.1);
        assertFailure(() -> timed.then(Mode.INVALID),
                IllegalArgumentException.class, "finite target");
        assertTrue(timed.then(Mode.IDLE).build() != null);

        SemanticScalarTasks.ReachedCancellationStep<Mode> cancellation =
                set.untilReachedBy(new TestPlant(command, true));
        assertFailure(() -> cancellation.cancelTo(Mode.INVALID),
                IllegalArgumentException.class, "finite target");
        SemanticScalarTasks.ReachedReadyStep<Mode> reached =
                cancellation.leaveRequestOnCancel();
        assertFailure(() -> reached.stableFor(-0.1),
                IllegalArgumentException.class, ">= 0");
        assertFailure(() -> reached.timeout(0.0),
                IllegalArgumentException.class, "> 0");

        SemanticScalarTasks.ReachedReadyStep<Mode> stable = reached.stableFor(0.1);
        assertFailure(() -> stable.stableFor(0.2),
                IllegalStateException.class, "already been answered");
        SemanticScalarTasks.ReachedReadyStep<Mode> timeout = reached.timeout(0.5);
        assertFailure(() -> timeout.timeout(1.0),
                IllegalStateException.class, "already been answered");
        assertTrue(stable.build() != null);
        assertTrue(timeout.build() != null);
        assertSame(initial, command.request());
    }

    private static SemanticScalarCommand<Mode> command() {
        return SemanticScalarCommand.create(
                Mode.IDLE, mode -> mode == Mode.ACTIVE ? 0.75 : 0.0);
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

    private static Set<String> publicDeclaredMethodNames(Class<?> type) {
        Set<String> names = new HashSet<String>();
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) names.add(method.getName());
        }
        return names;
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
            assertTrue("wrong exception: " + expected, expectedType.isInstance(expected));
            assertTrue("missing message part '" + messagePart + "' in " + expected,
                    expected.getMessage().contains(messagePart));
        }
    }

    private static final class TestPlant implements Plant {
        private final SemanticScalarCommand<?> command;
        private final boolean feedback;

        private TestPlant(SemanticScalarCommand<?> command, boolean feedback) {
            this.command = command;
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
        public boolean hasFeedback() {
            return feedback;
        }

        @Override
        public boolean carriesSemanticCommand(SemanticScalarCommand<?> candidate) {
            return command != null && command == candidate;
        }

        @Override
        public void stop() {
        }
    }
}
