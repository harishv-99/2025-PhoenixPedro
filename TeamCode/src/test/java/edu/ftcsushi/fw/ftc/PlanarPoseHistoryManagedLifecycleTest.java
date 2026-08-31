package edu.ftcsushi.fw.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Proves the documented history lifecycle inside one existing managed localization service. */
public final class PlanarPoseHistoryManagedLifecycleTest {

    @Test
    public void localizationOwnerClearsUpdatesRecordsAndPrecedesTimestampedConsumers() {
        List<String> events = new ArrayList<String>();
        LocalizationOwner localization = new LocalizationOwner(events);
        HistoryConsumer consumer = new HistoryConsumer(
                localization.history.lookupSource(),
                events
        );

        // Seed another clock to prove START clearing releases the old lifecycle binding.
        LoopClock priorClock = new LoopClock();
        priorClock.reset(-1.0);
        localization.estimator.publish(priorClock, -1.0);
        localization.history.recordCurrent(priorClock);

        RobotProgram program = new RobotProgram(telemetryProxy());
        program.beginInit(0.0);
        program.service(localization);
        program.service(consumer);
        program.finishConfiguration();

        program.start(10.0);
        program.loop(10.1);
        program.stop();

        assertEquals(Arrays.asList(5.0, 6.0), consumer.observedXInches);
        assertEquals(
                Arrays.asList(
                        "history.reset:start",
                        "localization.anchor",
                        "localization.update:start",
                        "history.record:start",
                        "consumer.start",
                        "localization.update:loop",
                        "history.record:loop",
                        "consumer.update",
                        "consumer.stop",
                        "localization.resource.stop",
                        "history.reset:stop"
                ),
                events
        );

        PlanarPoseHistory.Lookup afterStop = localization.history.lookupSource().getAt(
                consumer.lastClock,
                consumer.lastClock.nowTimestamp()
        );
        assertEquals(PlanarPoseHistory.Lookup.Kind.UNAVAILABLE, afterStop.kind());
        assertEquals(
                PlanarPoseHistory.Lookup.UnavailableReason.EMPTY,
                afterStop.unavailableReason()
        );
    }

    private static final class LocalizationOwner implements RobotProgram.Service {
        final RecordingTrajectoryEstimator estimator = new RecordingTrajectoryEstimator();
        final PlanarPoseHistory history = new PlanarPoseHistory(
                estimator,
                PlanarPoseHistory.Config.defaults()
        );
        final List<String> events;
        double currentXInches;

        LocalizationOwner(List<String> events) {
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            events.add("history.reset:start");
            history.reset();
            currentXInches = 5.0;
            events.add("localization.anchor");
            estimator.publish(clock, currentXInches);
            events.add("localization.update:start");
            estimator.update(clock);
            history.recordCurrent(clock);
            events.add("history.record:start");
        }

        @Override
        public void update(LoopClock clock) {
            currentXInches += 1.0;
            estimator.publish(clock, currentXInches);
            events.add("localization.update:loop");
            estimator.update(clock);
            history.recordCurrent(clock);
            events.add("history.record:loop");
        }

        @Override
        public void stop() {
            events.add("localization.resource.stop");
            history.reset();
            events.add("history.reset:stop");
        }
    }

    private static final class HistoryConsumer implements RobotProgram.Service {
        final TimeAwareSource<PlanarPoseHistory.Lookup> history;
        final List<String> events;
        final List<Double> observedXInches = new ArrayList<Double>();
        LoopClock lastClock;

        HistoryConsumer(TimeAwareSource<PlanarPoseHistory.Lookup> history,
                        List<String> events) {
            this.history = history;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            observe(clock);
            events.add("consumer.start");
        }

        @Override
        public void update(LoopClock clock) {
            observe(clock);
            events.add("consumer.update");
        }

        @Override
        public void stop() {
            events.add("consumer.stop");
        }

        private void observe(LoopClock clock) {
            lastClock = clock;
            PlanarPoseHistory.Lookup lookup = history.get(clock);
            assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, lookup.kind());
            assertTrue(lookup.isAvailable());
            observedXInches.add(lookup.fieldToRobotPose().xInches);
        }
    }

    private static final class RecordingTrajectoryEstimator
            implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        int updateCalls;

        void publish(LoopClock clock, double xInches) {
            estimate = new PoseEstimate(
                    new Pose3d(xInches, 0.0, 0.0, 0.0, 0.0, 0.0),
                    true,
                    1.0,
                    clock.nowTimestamp()
            );
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public long trajectorySegmentId() {
            return 0L;
        }
    }

    private static Telemetry telemetryProxy() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> defaultValue(method)
        );
    }

    private static Object defaultValue(Method method) {
        Class<?> type = method.getReturnType();
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
        throw new AssertionError("Unsupported primitive telemetry return type: " + type.getName());
    }
}
