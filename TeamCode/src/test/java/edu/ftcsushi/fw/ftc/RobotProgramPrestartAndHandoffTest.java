package edu.ftcsushi.fw.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies managed INIT policy and transactional STOP publication. */
public final class RobotProgramPrestartAndHandoffTest {

    @Test
    public void prestartUpdatesBeforeEveryInitFrameAndFreezesBeforeClockReset() {
        List<String> events = new ArrayList<>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        RobotProgram program = new RobotProgram(telemetry.proxy());
        RecordingPrestart prestart = new RecordingPrestart(events);
        RecordingService service = new RecordingService("service", events);
        RecordingOutput output = new RecordingOutput("output", events);
        List<ClockSnapshot> presenterFrames = new ArrayList<>();

        program.beginInit(10.0);
        assertSame(prestart, program.prestart(prestart));
        program.service(service);
        program.output(output);
        program.presenter((clock, destination) -> {
            events.add("presenter");
            presenterFrames.add(ClockSnapshot.of(clock));
        });
        program.finishConfiguration();

        program.presentConfiguredInit();
        program.initLoop(10.25);

        assertEquals(
                Arrays.asList(
                        "prestart.update",
                        "presenter",
                        "telemetry.update",
                        "prestart.update",
                        "presenter",
                        "telemetry.update"
                ),
                events
        );
        assertEquals(
                Arrays.asList(
                        new ClockSnapshot(1L, 10.0, 0.0),
                        new ClockSnapshot(2L, 10.25, 0.25)
                ),
                prestart.updateFrames
        );
        assertEquals(prestart.updateFrames, presenterFrames);

        events.clear();
        program.start(20.0);

        assertEquals(
                Arrays.asList("prestart.freeze", "service.start", "output.update"),
                events
        );
        assertEquals(new ClockSnapshot(2L, 10.25, 0.25), prestart.freezeFrame);
        assertEquals(new ClockSnapshot(3L, 20.0, 0.0), ClockSnapshot.of(service.startClock));
        assertSame(service.startClock, output.updateClock);
        assertEquals(1, prestart.freezeCalls);

        program.stop();
    }

    @Test
    public void blockedStartKeepsEveryBehaviorOwnerInertAndPresentersVisible() {
        List<String> events = new ArrayList<>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        RobotProgram program = new RobotProgram(telemetry.proxy());
        RecordingPrestart prestart = new RecordingPrestart(events);
        prestart.disposition = RobotProgram.StartDisposition.BLOCKED;
        RecordingService service = new RecordingService("service", events);
        RecordingTask root = new RecordingTask("root", events);
        RecordingOutput output = new RecordingOutput("output", events);
        int[] bindingCalls = {0};
        List<ClockSnapshot> presenterFrames = new ArrayList<>();

        program.beginInit(5.0);
        program.prestart(prestart);
        program.service(service);
        program.callbackBindings().whileHigh(
                BooleanSource.constant(true),
                () -> bindingCalls[0]++);
        program.rootTask(root);
        program.output(output);
        program.presenter((clock, destination) -> {
            events.add("presenter");
            presenterFrames.add(ClockSnapshot.of(clock));
        });
        program.finishConfiguration();
        program.presentConfiguredInit();
        events.clear();

        program.start(20.0);
        program.loop(20.5);
        program.loop(21.0);

        assertEquals(
                Arrays.asList(
                        "prestart.freeze",
                        "presenter",
                        "telemetry.update",
                        "presenter",
                        "telemetry.update"
                ),
                events
        );
        assertEquals(0, service.startCalls);
        assertEquals(0, service.updateCalls);
        assertEquals(0, root.startCalls);
        assertEquals(0, root.updateCalls);
        assertEquals(0, output.updateCalls);
        assertEquals(0, bindingCalls[0]);
        assertEquals(
                Arrays.asList(
                        new ClockSnapshot(3L, 20.5, 0.5),
                        new ClockSnapshot(4L, 21.0, 0.5)
                ),
                presenterFrames.subList(1, 3)
        );

        events.clear();
        program.stop();
        assertEquals(Arrays.asList("output.stop", "service.stop"), events);
        assertEquals(0, root.cancelCalls);
    }

    @Test
    public void prestartIsSingleAndNullDispositionFailsClosed() {
        List<String> events = new ArrayList<>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        RobotProgram program = new RobotProgram(telemetry.proxy());
        RecordingPrestart accepted = new RecordingPrestart(events);
        RecordingPrestart rejected = new RecordingPrestart(events);
        RecordingOutput output = new RecordingOutput("output", events);

        program.beginInit(0.0);
        assertTrue(expectRuntimeException(() -> program.prestart(null))
                instanceof NullPointerException);
        assertSame(accepted, program.prestart(accepted));
        RuntimeException duplicate = expectRuntimeException(() -> program.prestart(rejected));
        assertTrue(duplicate.getMessage().contains("already has a prestart"));
        program.output(output);
        program.finishConfiguration();
        program.presentConfiguredInit();
        accepted.disposition = null;

        RuntimeException failure = expectRuntimeException(() -> program.start(1.0));
        assertTrue(failure instanceof NullPointerException);
        assertTrue(failure.getMessage().contains("return READY or BLOCKED"));
        RuntimeException retained = program.stopAfterFailure(failure);

        assertSame(failure, retained);
        assertEquals(1, accepted.freezeCalls);
        assertEquals(0, rejected.updateCalls);
        assertEquals(0, rejected.freezeCalls);
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void stopHandoffIsSingleValidatesBeforeMutationAndInvalidatesImmediately() {
        List<String> events = new ArrayList<>();
        ProgramFixture fixture = new ProgramFixture(events);
        RobotProgram program = fixture.program;
        int[] acceptedInvalidations = {0};
        int[] rejectedInvalidations = {0};

        assertTrue(expectRuntimeException(() -> program.stopHandoff(
                null,
                value -> { },
                () -> { }
        )) instanceof NullPointerException);
        assertTrue(expectRuntimeException(() -> program.stopHandoff(
                () -> "value",
                null,
                () -> { }
        )) instanceof NullPointerException);
        assertTrue(expectRuntimeException(() -> program.stopHandoff(
                () -> "value",
                value -> { },
                null
        )) instanceof NullPointerException);

        program.stopHandoff(
                () -> "accepted",
                value -> { },
                () -> acceptedInvalidations[0]++
        );
        assertEquals(1, acceptedInvalidations[0]);

        RuntimeException duplicate = expectRuntimeException(() -> program.stopHandoff(
                () -> "rejected",
                value -> { },
                () -> rejectedInvalidations[0]++
        ));
        assertTrue(duplicate.getMessage().contains("already has a stop handoff"));
        assertEquals(0, rejectedInvalidations[0]);

        program.finishConfiguration();
        assertTrue(expectRuntimeException(() -> program.stopHandoff(
                () -> "late",
                value -> { },
                () -> rejectedInvalidations[0]++
        )).getMessage().contains("after configure(program) returns"));
        assertEquals(0, rejectedInvalidations[0]);

        program.stop();
        assertEquals(2, acceptedInvalidations[0]);
    }

    @Test
    public void activeStopCapturesAfterTerminalizationAndPublishesAfterCleanup() {
        List<String> events = new ArrayList<>();
        ProgramFixture fixture = new ProgramFixture(events);
        RobotProgram program = fixture.program;
        RobotProgram[] retained = {program};

        program.stopHandoff(
                () -> {
                    events.add("handoff.capture");
                    retained[0].stop();
                    return "field-pose";
                },
                value -> events.add("handoff.publish:" + value),
                () -> events.add("handoff.invalidate")
        );
        finishAndStart(program, events);

        program.stop();

        assertEquals(
                Arrays.asList(
                        "handoff.capture",
                        "output.stop",
                        "service.stop",
                        "handoff.publish:field-pose"
                ),
                events
        );
    }

    @Test
    public void nonActiveAndBlockedStopsInvalidateWithoutCaptureOrPublication() {
        for (boolean blocked : new boolean[]{false, true}) {
            List<String> events = new ArrayList<>();
            ProgramFixture fixture = new ProgramFixture(events);
            RobotProgram program = fixture.program;
            int[] captures = {0};
            int[] publications = {0};
            int[] invalidations = {0};
            program.stopHandoff(
                    () -> {
                        captures[0]++;
                        return "value";
                    },
                    value -> publications[0]++,
                    () -> invalidations[0]++
            );
            if (blocked) {
                RecordingPrestart prestart = new RecordingPrestart(events);
                prestart.disposition = RobotProgram.StartDisposition.BLOCKED;
                program.prestart(prestart);
                program.finishConfiguration();
                program.presentConfiguredInit();
                program.start(1.0);
            } else {
                program.finishConfiguration();
                program.presentConfiguredInit();
            }
            events.clear();

            program.stop();

            assertEquals(blocked ? "blocked" : "before START", 0, captures[0]);
            assertEquals(blocked ? "blocked" : "before START", 0, publications[0]);
            assertEquals(blocked ? "blocked" : "before START", 2, invalidations[0]);
            assertEquals(
                    blocked ? "blocked" : "before START",
                    Arrays.asList("output.stop", "service.stop"),
                    events
            );
        }
    }

    @Test
    public void captureFailureRemainsPrimaryAcrossCleanupAndInvalidationFailures() {
        List<String> events = new ArrayList<>();
        RuntimeException captureFailure = new IllegalStateException("capture");
        RuntimeException outputFailure = new IllegalArgumentException("output stop");
        RuntimeException serviceFailure = new IllegalArgumentException("service stop");
        RuntimeException invalidationFailure = new IllegalArgumentException("invalidate");
        ProgramFixture fixture = new ProgramFixture(events);
        RobotProgram program = fixture.program;
        RecordingOutput output = fixture.output;
        RecordingService service = fixture.service;
        output.stopFailure = outputFailure;
        service.stopFailure = serviceFailure;
        int[] invalidations = {0};

        program.stopHandoff(
                () -> {
                    events.add("handoff.capture");
                    throw captureFailure;
                },
                value -> events.add("handoff.publish"),
                () -> {
                    invalidations[0]++;
                    events.add("handoff.invalidate");
                    if (invalidations[0] > 1) {
                        throw invalidationFailure;
                    }
                }
        );
        finishAndStart(program, events);

        RuntimeException actual = expectRuntimeException(program::stop);

        assertSame(captureFailure, actual);
        assertArrayEquals(
                new Throwable[]{outputFailure, serviceFailure, invalidationFailure},
                actual.getSuppressed()
        );
        assertEquals(
                Arrays.asList(
                        "handoff.capture",
                        "output.stop",
                        "service.stop",
                        "handoff.invalidate"
                ),
                events
        );
    }

    @Test
    public void cleanupFailurePreventsPublicationAndInvalidatesLast() {
        List<String> events = new ArrayList<>();
        RuntimeException outputFailure = new IllegalStateException("output stop");
        RuntimeException serviceFailure = new IllegalArgumentException("service stop");
        RuntimeException invalidationFailure = new IllegalArgumentException("invalidate");
        ProgramFixture fixture = new ProgramFixture(events);
        RobotProgram program = fixture.program;
        RecordingOutput output = fixture.output;
        RecordingService service = fixture.service;
        output.stopFailure = outputFailure;
        service.stopFailure = serviceFailure;
        int[] publications = {0};
        int[] invalidations = {0};

        program.stopHandoff(
                () -> {
                    events.add("handoff.capture");
                    return "value";
                },
                value -> publications[0]++,
                () -> {
                    invalidations[0]++;
                    events.add("handoff.invalidate");
                    if (invalidations[0] > 1) {
                        throw invalidationFailure;
                    }
                }
        );
        finishAndStart(program, events);

        RuntimeException actual = expectRuntimeException(program::stop);

        assertSame(outputFailure, actual);
        assertArrayEquals(
                new Throwable[]{serviceFailure, invalidationFailure},
                actual.getSuppressed()
        );
        assertEquals(0, publications[0]);
        assertEquals(
                Arrays.asList(
                        "handoff.capture",
                        "output.stop",
                        "service.stop",
                        "handoff.invalidate"
                ),
                events
        );
    }

    @Test
    public void publicationFailureInvalidatesAfterSuccessfulCleanup() {
        List<String> events = new ArrayList<>();
        RuntimeException publicationFailure = new IllegalStateException("publish");
        RuntimeException invalidationFailure = new IllegalArgumentException("invalidate");
        ProgramFixture fixture = new ProgramFixture(events);
        RobotProgram program = fixture.program;
        int[] invalidations = {0};

        program.stopHandoff(
                () -> {
                    events.add("handoff.capture");
                    return "value";
                },
                value -> {
                    events.add("handoff.publish");
                    throw publicationFailure;
                },
                () -> {
                    invalidations[0]++;
                    events.add("handoff.invalidate");
                    if (invalidations[0] > 1) {
                        throw invalidationFailure;
                    }
                }
        );
        finishAndStart(program, events);

        RuntimeException actual = expectRuntimeException(program::stop);

        assertSame(publicationFailure, actual);
        assertArrayEquals(new Throwable[]{invalidationFailure}, actual.getSuppressed());
        assertEquals(
                Arrays.asList(
                        "handoff.capture",
                        "output.stop",
                        "service.stop",
                        "handoff.publish",
                        "handoff.invalidate"
                ),
                events
        );
    }

    @Test
    public void lifecycleFailureNeverCapturesOrPublishesAndInvalidatesAfterCleanup() {
        List<String> events = new ArrayList<>();
        RuntimeException primary = new IllegalStateException("active failure");
        RuntimeException outputFailure = new IllegalArgumentException("output stop");
        RuntimeException invalidationFailure = new IllegalArgumentException("invalidate");
        ProgramFixture fixture = new ProgramFixture(events);
        RobotProgram program = fixture.program;
        fixture.output.stopFailure = outputFailure;
        int[] captures = {0};
        int[] publications = {0};
        int[] invalidations = {0};
        program.stopHandoff(
                () -> {
                    captures[0]++;
                    return "value";
                },
                value -> publications[0]++,
                () -> {
                    invalidations[0]++;
                    events.add("handoff.invalidate");
                    if (invalidations[0] > 1) {
                        throw invalidationFailure;
                    }
                }
        );
        finishAndStart(program, events);

        RuntimeException actual = program.stopAfterFailure(primary);

        assertSame(primary, actual);
        assertArrayEquals(
                new Throwable[]{outputFailure, invalidationFailure},
                actual.getSuppressed()
        );
        assertEquals(0, captures[0]);
        assertEquals(0, publications[0]);
        assertEquals(2, invalidations[0]);
        assertEquals(
                Arrays.asList("output.stop", "service.stop", "handoff.invalidate"),
                events
        );
    }

    @Test
    public void bindingReentrantStopDefersCaptureAndCleanupUntilTraversalUnwinds() {
        List<String> events = new ArrayList<>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        RobotProgram program = new RobotProgram(telemetry.proxy());
        RecordingService service = new RecordingService("service", events);
        RecordingOutput output = new RecordingOutput("output", events);
        RecordingTask root = new RecordingTask("root", events);
        boolean[] bindingUnwound = {false};
        boolean[] captureObservedUnwound = {false};
        boolean[] cleanupObservedUnwound = {false};

        program.beginInit(0.0);
        program.service(service);
        program.callbackBindings().whileHigh(BooleanSource.constant(true), () -> {
            events.add("binding.action");
            try {
                program.stop();
            } finally {
                bindingUnwound[0] = true;
                events.add("binding.unwound");
            }
            events.add("binding.after");
        });
        program.rootTask(root);
        program.output(output);
        output.stopAction = () -> cleanupObservedUnwound[0] = bindingUnwound[0];
        program.stopHandoff(
                () -> {
                    captureObservedUnwound[0] = bindingUnwound[0];
                    events.add("handoff.capture");
                    return "value";
                },
                value -> events.add("handoff.publish"),
                () -> events.add("handoff.invalidate")
        );
        program.finishConfiguration();
        program.presentConfiguredInit();
        program.start(1.0);
        events.clear();

        program.loop(1.1);

        assertEquals(
                Arrays.asList(
                        "service.update",
                        "binding.action",
                        "binding.unwound",
                        "handoff.capture",
                        "root.cancel",
                        "output.stop",
                        "service.stop",
                        "handoff.publish"
                ),
                events
        );
        assertTrue(captureObservedUnwound[0]);
        assertTrue(cleanupObservedUnwound[0]);
        assertTrue(!events.contains("binding.after"));

        int eventCount = events.size();
        program.stop();
        assertEquals(eventCount, events.size());
    }

    @Test
    public void handoffErrorsEscapeWithoutBeingConvertedOrAggregated() {
        List<String> events = new ArrayList<>();
        ProgramFixture fixture = new ProgramFixture(events);
        RobotProgram program = fixture.program;
        AssertionError fatal = new AssertionError("fatal capture");
        int[] invalidations = {0};
        program.stopHandoff(
                () -> {
                    throw fatal;
                },
                value -> fail("must not publish"),
                () -> invalidations[0]++
        );
        finishAndStart(program, events);

        AssertionError actual = expectAssertionError(program::stop);

        assertSame(fatal, actual);
        assertEquals(0, fixture.output.stopCalls);
        assertEquals(0, fixture.service.stopCalls);
        assertEquals(1, invalidations[0]);
    }

    private static void finishAndStart(RobotProgram program, List<String> events) {
        program.finishConfiguration();
        program.presentConfiguredInit();
        program.start(1.0);
        events.clear();
    }

    private static RuntimeException expectRuntimeException(Runnable operation) {
        try {
            operation.run();
            fail("Expected RuntimeException");
            throw new AssertionError("unreachable");
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static AssertionError expectAssertionError(Runnable operation) {
        try {
            operation.run();
            fail("Expected AssertionError");
            throw new AssertionError("unreachable");
        } catch (AssertionError expected) {
            return expected;
        }
    }

    private static final class ProgramFixture {
        final RecordingTelemetry telemetry;
        final RobotProgram program;
        final RecordingService service;
        final RecordingOutput output;

        ProgramFixture(List<String> events) {
            telemetry = new RecordingTelemetry(events);
            program = new RobotProgram(telemetry.proxy());
            service = new RecordingService("service", events);
            output = new RecordingOutput("output", events);
            program.beginInit(0.0);
            program.service(service);
            program.output(output);
        }
    }

    private static final class RecordingPrestart implements RobotProgram.Prestart {
        final List<String> events;
        final List<ClockSnapshot> updateFrames = new ArrayList<>();
        RobotProgram.StartDisposition disposition = RobotProgram.StartDisposition.READY;
        LoopClock lastUpdateClock;
        ClockSnapshot freezeFrame;
        int updateCalls;
        int freezeCalls;

        RecordingPrestart(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            lastUpdateClock = clock;
            updateFrames.add(ClockSnapshot.of(clock));
            events.add("prestart.update");
        }

        @Override
        public RobotProgram.StartDisposition freezeForStart() {
            freezeCalls++;
            freezeFrame = lastUpdateClock == null
                    ? null
                    : ClockSnapshot.of(lastUpdateClock);
            events.add("prestart.freeze");
            return disposition;
        }
    }

    private static final class RecordingService implements RobotProgram.Service {
        final String name;
        final List<String> events;
        RuntimeException stopFailure;
        LoopClock startClock;
        int startCalls;
        int updateCalls;
        int stopCalls;

        RecordingService(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            startCalls++;
            startClock = clock;
            events.add(name + ".start");
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            events.add(name + ".update");
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add(name + ".stop");
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingOutput implements RobotProgram.Output {
        final String name;
        final List<String> events;
        Runnable stopAction;
        RuntimeException stopFailure;
        LoopClock updateClock;
        int updateCalls;
        int stopCalls;

        RecordingOutput(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            updateClock = clock;
            events.add(name + ".update");
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add(name + ".stop");
            if (stopAction != null) {
                stopAction.run();
            }
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingTask implements Task {
        final String name;
        final List<String> events;
        boolean started;
        boolean cancelled;
        int startCalls;
        int updateCalls;
        int cancelCalls;

        RecordingTask(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            startCalls++;
            events.add(name + ".start");
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            events.add(name + ".update");
        }

        @Override
        public void cancel() {
            if (!started || cancelled) {
                return;
            }
            cancelled = true;
            cancelCalls++;
            events.add(name + ".cancel");
        }

        @Override
        public boolean isComplete() {
            return cancelled;
        }

        @Override
        public TaskOutcome getOutcome() {
            return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }

    private static final class ClockSnapshot {
        final long cycle;
        final double nowSec;
        final double dtSec;

        ClockSnapshot(long cycle, double nowSec, double dtSec) {
            this.cycle = cycle;
            this.nowSec = nowSec;
            this.dtSec = dtSec;
        }

        static ClockSnapshot of(LoopClock clock) {
            return new ClockSnapshot(clock.cycle(), clock.nowSec(), clock.dtSec());
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof ClockSnapshot)) {
                return false;
            }
            ClockSnapshot that = (ClockSnapshot) other;
            return cycle == that.cycle
                    && Double.compare(nowSec, that.nowSec) == 0
                    && Double.compare(dtSec, that.dtSec) == 0;
        }

        @Override
        public int hashCode() {
            long nowBits = Double.doubleToLongBits(nowSec);
            long dtBits = Double.doubleToLongBits(dtSec);
            int result = (int) (cycle ^ (cycle >>> 32));
            result = 31 * result + (int) (nowBits ^ (nowBits >>> 32));
            result = 31 * result + (int) (dtBits ^ (dtBits >>> 32));
            return result;
        }

        @Override
        public String toString() {
            return "ClockSnapshot{" + cycle + ", " + nowSec + ", " + dtSec + '}';
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> events;
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );

        RecordingTelemetry(List<String> events) {
            this.events = events;
        }

        Telemetry proxy() {
            return proxy;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if ("update".equals(method.getName())) {
                events.add("telemetry.update");
            }
            return defaultValue(method.getReturnType());
        }

        private static Object defaultValue(Class<?> type) {
            if (!type.isPrimitive()) {
                return null;
            }
            if (type == boolean.class) {
                return false;
            }
            if (type == char.class) {
                return '\0';
            }
            if (type == byte.class) {
                return (byte) 0;
            }
            if (type == short.class) {
                return (short) 0;
            }
            if (type == int.class) {
                return 0;
            }
            if (type == long.class) {
                return 0L;
            }
            if (type == float.class) {
                return 0.0f;
            }
            if (type == double.class) {
                return 0.0;
            }
            return null;
        }
    }
}
