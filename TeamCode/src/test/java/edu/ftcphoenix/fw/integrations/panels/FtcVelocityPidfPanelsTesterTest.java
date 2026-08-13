package edu.ftcphoenix.fw.integrations.panels;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetStatus;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Segment, validation, evidence, and cleanup coverage for the generic Panels tuner. */
public final class FtcVelocityPidfPanelsTesterTest {

    @Test
    public void draftAloneIsInertAndColdApplyReadsBackBeforePlantTarget() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.events.clear();

        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0);
        fixture.tick(0.01, false, false, false);
        assertFalse(fixture.events.contains("controller.set"));
        assertEquals(0.0, fixture.plant.command.get(), 0.0);

        fixture.events.clear();
        fixture.pressA(0.02);
        assertBefore(fixture.events, "controller.set", "controller.getKF");
        assertBefore(fixture.events, "controller.getKF", "target=900.0");
        assertBefore(fixture.events, "target=900.0", "plant.update=900.0");
        assertEquals("COLD_START", fixture.telemetry.currentSegmentType());
    }

    @Test
    public void hotApplyChangesCompleteCandidateWithoutRequestingZero() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0.25);
        fixture.pressA(0.01);

        fixture.events.clear();
        fixture.draft.candidate = candidate(5, 6, 7, 8, 1200, 0.30);
        fixture.pressA(0.20);

        assertFalse(fixture.events.toString(), fixture.events.contains("target=0.0"));
        assertBefore(fixture.events, "controller.set", "controller.getKF");
        assertBefore(fixture.events, "controller.getKF", "target=1200.0");
        assertEquals("HOT_UPDATE", fixture.telemetry.currentSegmentType());
        assertGains(5, 6, 7, 8, fixture.controller.lastApplied);

        fixture.events.clear();
        fixture.tick(0.60, false, false, false);
        assertFalse("hot segment owns a fresh timer", fixture.events.contains("target=0.0"));
        fixture.tick(0.62, false, false, false);
        assertTrue(fixture.events.toString(), fixture.events.contains("target=0.0"));
    }

    @Test
    public void invalidHotCandidateLeavesRunningTargetAndTimerUnchanged() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0.50);
        fixture.pressA(0.01);

        fixture.events.clear();
        fixture.draft.candidate = candidate(9, 9, 9, 9, 3000, 2.0);
        fixture.pressA(0.20);
        assertFalse(fixture.events.contains("controller.set"));
        assertFalse(fixture.events.contains("target=3000.0"));
        assertEquals(900.0, fixture.plant.command.get(), 0.0);
        assertTrue(fixture.telemetry.value("Status").contains("REJECTED"));

        fixture.events.clear();
        fixture.tick(0.61, false, false, false);
        assertFalse(fixture.events.contains("target=0.0"));
        fixture.tick(0.63, false, false, false);
        assertTrue(fixture.events.contains("target=0.0"));
    }

    @Test
    public void nextColdStartRequiresFiniteTruthfulZeroFeedback() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0);
        fixture.pressA(0.01);

        fixture.plant.measurement = 180.0;
        fixture.pressB(0.20);
        fixture.draft.candidate = candidate(5, 6, 7, 8, 1100, 0);
        fixture.events.clear();
        fixture.pressA(0.23);
        assertFalse(fixture.events.contains("controller.set"));
        assertEquals(0.0, fixture.plant.command.get(), 0.0);

        fixture.plant.measurement = Double.NaN;
        fixture.tick(0.36, false, false, false);
        fixture.tick(0.37, false, false, false);
        assertFalse(fixture.events.contains("controller.set"));
        assertTrue(fixture.telemetry.value("Status").contains("feedback is unavailable"));
        assertFinite(fixture.telemetry,
                FtcVelocityPidfPanelsTester.GRAPH_MEASUREMENT,
                0.0);

        fixture.plant.measurement = 0.0;
        fixture.tick(0.38, false, false, false);
        assertFalse("cold gate uses the preceding Plant observation",
                fixture.events.contains("controller.set"));
        fixture.tick(0.39, false, false, false);
        assertTrue(fixture.events.toString(), fixture.events.contains("controller.set"));
        assertEquals(1100.0, fixture.plant.command.get(), 0.0);
    }

    @Test
    public void positiveDurationRequestsNonterminalZeroAtSegmentBoundary() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0.10);
        fixture.pressA(0.01);

        fixture.events.clear();
        fixture.tick(0.21, false, false, false);
        assertFalse(fixture.events.contains("target=0.0"));
        fixture.tick(0.23, false, false, false);
        assertTrue(fixture.events.contains("target=0.0"));
        assertEquals(0, fixture.plant.stopCount);
    }

    @Test
    public void zeroDurationKeepsSegmentActiveUntilAnExplicitSafetyAction() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0.0);
        fixture.pressA(0.01);

        fixture.events.clear();
        fixture.tick(100.0, false, false, false);

        assertFalse(fixture.events.toString(), fixture.events.contains("target=0.0"));
        assertEquals(900.0, fixture.plant.command.get(), 0.0);
        assertEquals(0, fixture.plant.stopCount);
    }

    @Test
    public void controllerFailureTerminallyStopsAndRestoresOnce() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0);
        fixture.controller.setFailure = new IllegalStateException("transport failed");

        try {
            fixture.pressA(0.01);
            fail("expected terminal failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
        }

        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.controller.restoreCount);
        fixture.tester.stop();
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.controller.restoreCount);
    }

    @Test
    public void illegalArgumentReadbackAfterSuccessfulApplyIsTerminalNotRejected() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0);
        fixture.controller.readFailure = new IllegalArgumentException("bad readback tuple");

        try {
            fixture.pressA(0.01);
            fail("expected terminal readback failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
            assertTrue(expected.getMessage().contains("reading back applied"));
        }

        assertGains(1, 2, 3, 4, fixture.controller.lastApplied);
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.controller.restoreCount);
    }

    @Test
    public void applyTimeIllegalArgumentRejectsCandidateWithoutStoppingRunningSegment() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0.50);
        fixture.pressA(0.01);
        fixture.events.clear();

        fixture.controller.setFailure = new IllegalArgumentException("kP outside domain");
        fixture.draft.candidate = candidate(5, 6, 7, 8, 1200, 2.0);
        fixture.pressA(0.20);

        assertTrue(fixture.telemetry.value("Status").contains("REJECTED"));
        assertEquals(900.0, fixture.plant.command.get(), 0.0);
        assertEquals(0, fixture.plant.stopCount);
        assertEquals(0, fixture.controller.restoreCount);

        fixture.events.clear();
        fixture.tick(0.61, false, false, false);
        assertFalse(fixture.events.contains("target=0.0"));
        fixture.tick(0.63, false, false, false);
        assertTrue("rejection must not restart the original timer",
                fixture.events.contains("target=0.0"));
    }

    @Test
    public void targetCommitFailureAfterApplyAlsoStopsAndRestores() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0);
        fixture.plant.command.setFailure = new IllegalStateException("target failed");

        try {
            fixture.pressA(0.01);
            fail("expected terminal failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
        }
        assertGains(1, 2, 3, 4, fixture.controller.lastApplied);
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.controller.restoreCount);
    }

    @Test
    public void graphIsFiniteAndAcceptedEvidenceUsesControllerReadback() {
        Fixture fixture = new Fixture();
        fixture.controller.readbackOffset = 0.125;
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0);
        fixture.pressA(0.01);

        assertFinite(fixture.telemetry, FtcVelocityPidfPanelsTester.GRAPH_SEGMENT_ID, 1.0);
        assertFinite(fixture.telemetry, FtcVelocityPidfPanelsTester.GRAPH_TARGET, 900.0);
        assertFinite(fixture.telemetry, FtcVelocityPidfPanelsTester.GRAPH_MEASUREMENT, 0.0);
        assertFinite(fixture.telemetry, FtcVelocityPidfPanelsTester.GRAPH_ERROR, 900.0);
        assertFinite(
                fixture.telemetry,
                FtcVelocityPidfPanelsTester.GRAPH_ABSOLUTE_SPEED_RATE,
                0.0);
        assertTrue(fixture.telemetry.lines.toString(),
                fixture.telemetry.lines.contains("kP = 1.125"));

        fixture.plant.measurement = Double.MAX_VALUE;
        fixture.tick(0.14, false, false, false);
        assertEquals("UNAVAILABLE", fixture.telemetry.value("Absolute-speed rate"));
        assertFinite(fixture.telemetry,
                FtcVelocityPidfPanelsTester.GRAPH_ABSOLUTE_SPEED_RATE,
                0.0);
    }

    @Test
    public void unstableDraftRejectsWithoutReplacingActiveSegment() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.draft.candidate = candidate(1, 2, 3, 4, 900, 0);
        fixture.pressA(0.01);
        fixture.events.clear();

        fixture.draft.candidate = candidate(5, 6, 7, 8, 1000, 2.0);
        fixture.tick(0.20, true, false, false);
        for (int i = 1; i <= 9; i++) {
            double target = i % 2 == 0 ? 1000 : 1100;
            fixture.draft.candidate = candidate(5, 6, 7, 8, target, 2.0);
            fixture.tick(0.20 + i * 0.09, false, false, false);
        }

        assertFalse(fixture.events.contains("controller.set"));
        assertEquals(900.0, fixture.plant.command.get(), 0.0);
        assertTrue(fixture.telemetry.value("Status").contains("REJECTED"));
    }

    @Test
    public void backAndNormalStopUseTerminalPlantThenControllerCleanupOrder() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.events.clear();

        fixture.tick(0.01, false, false, true);
        assertBefore(fixture.events, "plant.stop", "controller.restore");
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.controller.restoreCount);

        fixture.tester.stop();
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.controller.restoreCount);
    }

    @Test
    public void backDuringInitCannotBeRearmedByLaterStart() {
        Fixture fixture = new Fixture();
        fixture.events.clear();

        fixture.initTick(0.005, false);
        fixture.initTick(0.01, true);
        fixture.tester.start();
        fixture.tick(0.02, true, false, false);

        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.controller.restoreCount);
        assertFalse(fixture.events.toString(), fixture.events.contains("plant.update=0.0"));
        assertTrue(fixture.telemetry.value("Status").contains("STOPPED"));
    }

    @Test
    public void cleanupAttemptsRestoreAfterPlantStopFailure() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.events.clear();
        fixture.plant.stopFailure = new IllegalStateException("plant stop failed");
        fixture.controller.restoreFailure = new IllegalStateException("restore failed");

        try {
            fixture.tester.stop();
            fail("expected cleanup failure");
        } catch (RuntimeException expected) {
            assertTrue(expected.getMessage().contains("plant stop failed"));
            assertEquals(1, expected.getSuppressed().length);
            assertTrue(expected.getSuppressed()[0].getMessage().contains("restore failed"));
        }
        assertBefore(fixture.events, "plant.stop", "controller.restore");
    }

    @Test
    public void facadeRejectsIncompleteConfigurationBeforeHardwareAcquisition() {
        try {
            FtcPanelsTuners.velocityPidf(
                    "Velocity",
                    ScalarRange.unbounded(),
                    ignored -> null);
            fail("expected finite range rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("finite bounded"));
        }

        try {
            FtcPanelsTuners.velocityPidf(
                    "Velocity",
                    ScalarRange.bounded(0, 2000),
                    ignored -> null);
            fail("expected nonpositive lower-bound rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("minValue > 0"));
        }
    }

    @Test
    public void facadeConstructionIsInactiveAndReturnsTheConfiguredName() {
        final boolean[] plantFactoryCalled = {false};
        TeleOpTester tester = FtcPanelsTuners.velocityPidf(
                "Lift Velocity PIDF",
                ScalarRange.bounded(100, 2000),
                ignored -> {
                    plantFactoryCalled[0] = true;
                    return null;
                });

        assertEquals("Lift Velocity PIDF", tester.name());
        assertFalse("hardware acquisition belongs to tester init", plantFactoryCalled[0]);
    }

    @Test
    public void publicAcquisitionRejectsAHardwareNeutralPlantAndStopsIt() {
        List<String> events = new ArrayList<String>();
        RecordingPlant plant = new RecordingPlant(events);
        TeleOpTester tester = FtcPanelsTuners.velocityPidf(
                "Velocity PIDF",
                ScalarRange.bounded(100, 2000),
                ignored -> plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        try {
            tester.init(new TesterContext(
                    new HardwareMap(null, null),
                    new RecordingTelemetry().proxy,
                    new Gamepad(),
                    new Gamepad(),
                    clock));
            fail("expected FTC Plant identity rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("single-motor FTC device-managed velocity"));
        }

        assertEquals(1, plant.stopCount);
        assertTrue(events.toString(), events.contains("plant.stop"));
    }

    @Test
    public void testRangeMustFitAZeroCapableBoundPlantRange() {
        FtcVelocityPidfPanelsTester.requireTestRangeInsidePlant(
                ScalarRange.bounded(100, 2000),
                ScalarRange.bounded(0, 2500));

        try {
            FtcVelocityPidfPanelsTester.requireTestRangeInsidePlant(
                    ScalarRange.bounded(100, 2000),
                    ScalarRange.bounded(500, 2500));
            fail("expected missing-zero rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("zero stop target"));
        }

        try {
            FtcVelocityPidfPanelsTester.requireTestRangeInsidePlant(
                    ScalarRange.bounded(100, 2600),
                    ScalarRange.bounded(0, 2500));
            fail("expected endpoint rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("must lie inside"));
        }
    }

    private static FtcVelocityPidfPanelsTester.Candidate candidate(
            double kP, double kI, double kD, double kF, double target, double autoStop) {
        return new FtcVelocityPidfPanelsTester.Candidate(
                kP, kI, kD, kF, target, autoStop);
    }

    private static void assertBefore(List<String> events, String first, String second) {
        int firstIndex = events.indexOf(first);
        int secondIndex = events.indexOf(second);
        assertTrue(first + " missing from " + events, firstIndex >= 0);
        assertTrue(second + " missing from " + events, secondIndex >= 0);
        assertTrue(first + " must precede " + second + " in " + events,
                firstIndex < secondIndex);
    }

    private static void assertGains(double kP, double kI, double kD, double kF,
                                    FtcVelocityPidfPanelsTester.Gains actual) {
        assertEquals(kP, actual.kP, 0.0);
        assertEquals(kI, actual.kI, 0.0);
        assertEquals(kD, actual.kD, 0.0);
        assertEquals(kF, actual.kF, 0.0);
    }

    private static void assertFinite(RecordingTelemetry telemetry, String key, double expected) {
        double actual = Double.parseDouble(telemetry.value(key));
        assertTrue(key + " must be finite", Double.isFinite(actual));
        assertEquals(expected, actual, 0.0);
    }

    private static final class Fixture {
        final List<String> events = new ArrayList<String>();
        final RecordingPlant plant = new RecordingPlant(events);
        final RecordingController controller = new RecordingController(events);
        final RecordingDraft draft = new RecordingDraft();
        final RecordingTelemetry telemetry = new RecordingTelemetry();
        final Gamepad gamepad = new Gamepad();
        final LoopClock clock = new LoopClock();
        final FtcVelocityPidfPanelsTester tester = new FtcVelocityPidfPanelsTester(
                "Velocity PIDF",
                ScalarRange.bounded(700, 2000),
                plant,
                controller,
                draft);

        Fixture() {
            clock.reset(0.0);
            tester.init(new TesterContext(
                    new HardwareMap(null, null),
                    telemetry.proxy,
                    gamepad,
                    new Gamepad(),
                    clock));
        }

        void start() {
            tester.start();
            tick(0.001, false, false, false);
            events.clear();
        }

        void pressA(double timeSec) {
            tick(timeSec, true, false, false);
            tick(timeSec + 0.01, false, false, false);
            tick(timeSec + 0.11, false, false, false);
        }

        void pressB(double timeSec) {
            tick(timeSec, false, true, false);
            tick(timeSec + 0.01, false, false, false);
        }

        void initTick(double timeSec, boolean back) {
            gamepad.back = back;
            gamepad.share = back;
            clock.update(timeSec);
            tester.initLoop(clock.dtSec());
        }

        void tick(double timeSec, boolean a, boolean b, boolean back) {
            gamepad.a = a;
            gamepad.cross = a;
            gamepad.b = b;
            gamepad.circle = b;
            gamepad.back = back;
            gamepad.share = back;
            clock.update(timeSec);
            tester.loop(clock.dtSec());
        }
    }

    private static final class RecordingDraft implements FtcVelocityPidfPanelsTester.DraftPort {
        FtcVelocityPidfPanelsTester.Candidate candidate = candidate(0, 0, 0, 0, 700, 5);

        @Override
        public FtcVelocityPidfPanelsTester.Candidate read() {
            return candidate;
        }

        @Override
        public void seedAndRefresh(FtcVelocityPidfPanelsTester.Gains gains,
                                   double target,
                                   double autoStopAfterSec) {
            candidate = new FtcVelocityPidfPanelsTester.Candidate(
                    gains, target, autoStopAfterSec);
        }
    }

    private static final class RecordingController
            implements FtcVelocityPidfPanelsTester.ControllerSession {
        final List<String> events;
        double kP;
        double kI;
        double kD;
        double kF;
        double readbackOffset;
        RuntimeException setFailure;
        RuntimeException readFailure;
        RuntimeException restoreFailure;
        FtcVelocityPidfPanelsTester.Gains lastApplied;
        int restoreCount;

        RecordingController(List<String> events) {
            this.events = events;
        }

        @Override
        public void apply(FtcVelocityPidfPanelsTester.Gains gains) {
            events.add("controller.set");
            if (setFailure != null) {
                throw setFailure;
            }
            lastApplied = gains;
            kP = gains.kP + readbackOffset;
            kI = gains.kI + readbackOffset;
            kD = gains.kD + readbackOffset;
            kF = gains.kF + readbackOffset;
        }

        @Override
        public FtcVelocityPidfPanelsTester.Gains readback() {
            if (readFailure != null) {
                throw readFailure;
            }
            events.add("controller.getKP");
            events.add("controller.getKI");
            events.add("controller.getKD");
            events.add("controller.getKF");
            return new FtcVelocityPidfPanelsTester.Gains(kP, kI, kD, kF);
        }

        @Override
        public void restoreInitial() {
            events.add("controller.restore");
            restoreCount++;
            if (restoreFailure != null) {
                throw restoreFailure;
            }
        }
    }

    private static final class RecordingPlant implements Plant {
        final RecordingTarget command;
        final List<String> events;
        double requested;
        double measurement;
        int stopCount;
        RuntimeException stopFailure;

        RecordingPlant(List<String> events) {
            this.events = events;
            command = new RecordingTarget(events);
        }

        @Override
        public void update(LoopClock clock) {
            requested = command.get();
            events.add("plant.update=" + requested);
        }

        @Override
        public double getRequestedTarget() {
            return requested;
        }

        @Override
        public double getAppliedTarget() {
            return requested;
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public double getMeasurement() {
            return measurement;
        }

        @Override
        public boolean atTarget() {
            return atTarget(requested);
        }

        @Override
        public boolean atTarget(double target) {
            return Double.isFinite(measurement)
                    && requested == target
                    && Math.abs(measurement - target) <= 50.0;
        }

        @Override
        public boolean hasCommandTarget() {
            return true;
        }

        @Override
        public ScalarTarget commandTarget() {
            return command;
        }

        @Override
        public void stop() {
            events.add("plant.stop");
            stopCount++;
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingTarget implements ScalarTarget {
        private final List<String> events;
        private double value;
        private RuntimeException setFailure;

        RecordingTarget(List<String> events) {
            this.events = events;
        }

        @Override
        public void set(double value) {
            if (setFailure != null) {
                throw setFailure;
            }
            this.value = value;
            events.add("target=" + value);
        }

        @Override
        public double get() {
            return value;
        }

        @Override
        public void reset() {
            value = 0.0;
        }
    }

    private static final class RecordingTelemetry {
        final Map<String, String> data = new LinkedHashMap<String, String>();
        final List<String> lines = new ArrayList<String>();
        final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (instance, method, args) -> {
                    String name = method.getName();
                    if (method.getDeclaringClass() == Object.class) {
                        if ("toString".equals(name)) return "RecordingTelemetry";
                        if ("hashCode".equals(name)) return System.identityHashCode(instance);
                        if ("equals".equals(name)) return instance == args[0];
                    }
                    if ("clearAll".equals(name)) {
                        data.clear();
                        lines.clear();
                        return null;
                    }
                    if ("addData".equals(name)) {
                        data.put(String.valueOf(args[0]), String.valueOf(args[1]));
                        return null;
                    }
                    if ("addLine".equals(name)) {
                        lines.add(args == null || args.length == 0 ? "" : String.valueOf(args[0]));
                        return null;
                    }
                    Class<?> returnType = method.getReturnType();
                    if (returnType == boolean.class) return true;
                    if (returnType == int.class) return 0;
                    return null;
                });

        String value(String key) {
            return data.get(key);
        }

        String currentSegmentType() {
            String value = value("Current segment");
            if (value == null) return null;
            if (value.contains("HOT_UPDATE")) return "HOT_UPDATE";
            if (value.contains("COLD_START")) return "COLD_START";
            return value;
        }
    }
}
