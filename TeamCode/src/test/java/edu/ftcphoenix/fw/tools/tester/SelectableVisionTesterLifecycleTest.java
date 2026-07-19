package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcphoenix.fw.tools.tester.localization.PinpointAprilTagFusionLocalizationTester;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused fail-stop coverage for testers that let operators replace an AprilTag vision lane. */
public final class SelectableVisionTesterLifecycleTest {

    @Test
    public void postOpenRuntimeFailureClosesOnceAndConfirmedCleanupAllowsRetry() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException firstFailure = new IllegalStateException(kind + " first setup");
            RuntimeException secondFailure = new IllegalArgumentException(kind + " second setup");
            LaneProbe firstLane = LaneProbe.failingSetup(firstFailure, null);
            LaneProbe secondLane = LaneProbe.failingSetup(secondFailure, null);
            OpenProbe opener = new OpenProbe(firstLane, secondLane);
            TeleOpTester owner = kind.create(opener);

            owner.init(context());

            assertEquals(kind.toString(), 1, opener.openCount);
            assertEquals(kind.toString(), 1, firstLane.closeCount);
            assertSame(kind.toString(), firstFailure, retainedFailure(owner));
            assertFalse(kind + " should allow leaving after confirmed cleanup", owner.onBackPressed());

            invokeEnsure(kind, owner);

            assertEquals(kind.toString(), 2, opener.openCount);
            assertEquals(kind.toString(), 1, secondLane.closeCount);
            assertSame(kind.toString(), secondFailure, retainedFailure(owner));

            owner.stop();
            owner.stop();
            assertEquals(kind.toString(), 1, firstLane.closeCount);
            assertEquals(kind.toString(), 1, secondLane.closeCount);
        }
    }

    @Test
    public void postOpenFailureRetainsPrimaryAndSuppressesFailedCleanup() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException primary = new IllegalStateException(kind + " setup");
            RuntimeException cleanup = new IllegalArgumentException(kind + " close");
            LaneProbe lane = LaneProbe.failingSetup(primary, cleanup);
            OpenProbe opener = new OpenProbe(lane);
            TeleOpTester owner = kind.create(opener);

            owner.init(context());

            assertSame(kind.toString(), primary, retainedFailure(owner));
            assertEquals(kind.toString(), 1, primary.getSuppressed().length);
            assertSame(kind.toString(), cleanup, primary.getSuppressed()[0]);
            assertEquals(kind.toString(), 1, lane.closeCount);
            assertTrue(kind + " must consume BACK while ownership is uncertain",
                    owner.onBackPressed());

            invokeEnsure(kind, owner);
            owner.stop();
            owner.stop();

            assertEquals(kind + " must block replacement", 1, opener.openCount);
            assertEquals(kind + " must not retry close", 1, lane.closeCount);
        }
    }

    @Test
    public void closingTransitionBlocksReentrantOpen() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException setup = new IllegalStateException(kind + " setup");
            LaneProbe lane = LaneProbe.failingSetup(setup, null);
            OpenProbe opener = new OpenProbe(lane);
            TeleOpTester owner = kind.create(opener);
            lane.duringClose = () -> invokeEnsure(kind, owner);

            owner.init(context());

            assertEquals(kind + " reentrant close must not open a replacement", 1, opener.openCount);
            assertEquals(kind.toString(), 1, lane.closeCount);
            assertFalse(kind + " may retry after close returns normally",
                    booleanField(owner, kind.closingField));
        }
    }

    @Test
    public void reentrantStopDuringCloseKeepsTerminalPrecedence() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException setup = new IllegalStateException(kind + " setup");
            LaneProbe firstLane = LaneProbe.failingSetup(setup, null);
            LaneProbe replacement = LaneProbe.failingSetup(
                    new IllegalStateException(kind + " replacement"),
                    null);
            OpenProbe opener = new OpenProbe(firstLane, replacement);
            TeleOpTester owner = kind.create(opener);
            firstLane.duringClose = owner::stop;

            owner.init(context());
            invokeEnsure(kind, owner);

            assertEquals(kind + " STOP must block a replacement open", 1, opener.openCount);
            assertEquals(kind.toString(), 1, firstLane.closeCount);
            assertTrue(kind + " terminal owner must consume BACK", owner.onBackPressed());
            owner.stop();
            assertEquals(kind + " repeated STOP must not retry close", 1, firstLane.closeCount);
        }
    }

    @Test
    public void activeBackDetachesBeforeCloseAndNeverRetriesFailedClose() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException cleanup = new IllegalStateException(kind + " active close");
            LaneProbe lane = LaneProbe.open(cleanup);
            TeleOpTester owner = kind.create(new OpenProbe());
            setField(owner, kind.laneField, lane);
            setBooleanField(owner, kind.readyField, true);
            lane.duringClose = () -> assertTrue(
                    kind + " must consume reentrant BACK during close",
                    owner.onBackPressed());

            assertTrue(kind.toString(), owner.onBackPressed());
            assertSame(kind.toString(), cleanup, retainedFailure(owner));
            assertEquals(kind.toString(), 1, lane.closeCount);
            assertTrue(kind + " must remain failed closed", owner.onBackPressed());

            owner.stop();
            owner.stop();
            assertEquals(kind + " must not retry the detached lane", 1, lane.closeCount);
        }
    }

    @Test
    public void finalStopDetachesBeforeCloseAndRepeatedStopDoesNotRetry() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException cleanup = new IllegalStateException(kind + " stop close");
            LaneProbe lane = LaneProbe.open(cleanup);
            TeleOpTester owner = kind.create(new OpenProbe());
            setField(owner, kind.laneField, lane);
            setBooleanField(owner, kind.readyField, true);

            try {
                owner.stop();
                fail("Expected " + kind + " stop failure");
            } catch (RuntimeException actual) {
                assertSame(kind.toString(), cleanup, actual);
            }

            owner.stop();
            assertEquals(kind + " must not retry final close", 1, lane.closeCount);
        }
    }

    @Test
    public void errorsAreNotCaughtAndPartiallyOpenedLaneCanStillBeStoppedOnce() {
        for (OwnerKind kind : OwnerKind.values()) {
            AssertionError error = new AssertionError(kind + " setup error");
            LaneProbe lane = LaneProbe.failingSetup(error);
            TeleOpTester owner = kind.create(new OpenProbe(lane));

            try {
                owner.init(context());
                fail("Expected " + kind + " Error");
            } catch (AssertionError actual) {
                assertSame(kind.toString(), error, actual);
            }

            assertEquals(kind.toString(), 0, lane.closeCount);
            owner.stop();
            owner.stop();
            assertEquals(kind + " must stop the retained partial lane once", 1, lane.closeCount);
        }
    }

    private enum OwnerKind {
        CAMERA_MOUNT("ensureVisionReady", "visionLane", "visionReady"),
        APRILTAG_LOCALIZATION("ensureVisionReady", "visionLane", "visionReady"),
        PINPOINT_FUSION("ensureReady", "visionLane", "ready");

        private final String ensureMethod;
        private final String laneField;
        private final String readyField;
        private final String closingField = "visionClosingOrTerminal";

        OwnerKind(String ensureMethod, String laneField, String readyField) {
            this.ensureMethod = ensureMethod;
            this.laneField = laneField;
            this.readyField = readyField;
        }

        TeleOpTester create(OpenProbe opener) {
            Function<String, AprilTagVisionLaneFactory> builder = ignored -> opener.factory();
            switch (this) {
                case CAMERA_MOUNT:
                    return new CameraMountCalibrator(
                            "vision",
                            HardwareDevice.class,
                            "Vision",
                            builder,
                            new SimpleTagLayout(),
                            0.35
                    );
                case APRILTAG_LOCALIZATION:
                    return new AprilTagLocalizationTester(
                            "vision",
                            HardwareDevice.class,
                            "Vision",
                            builder,
                            new SimpleTagLayout(),
                            null,
                            0.35
                    );
                case PINPOINT_FUSION:
                    return new PinpointAprilTagFusionLocalizationTester(
                            "vision",
                            HardwareDevice.class,
                            "Vision",
                            builder,
                            FtcOdometryAprilTagLocalizationLane.Config.defaults(),
                            new SimpleTagLayout()
                    );
                default:
                    throw new AssertionError(this);
            }
        }
    }

    private static final class OpenProbe {
        private final Deque<LaneProbe> lanes;
        private int openCount;

        OpenProbe(LaneProbe... lanes) {
            this.lanes = new ArrayDeque<>(Arrays.asList(lanes));
        }

        AprilTagVisionLaneFactory factory() {
            return hardwareMap -> {
                openCount++;
                LaneProbe lane = lanes.pollFirst();
                if (lane == null) {
                    throw new IllegalStateException("No queued test lane");
                }
                return lane;
            };
        }
    }

    private static final class LaneProbe implements AprilTagVisionLane {
        private final RuntimeException setupFailure;
        private final Error setupError;
        private final RuntimeException closeFailure;
        private int closeCount;
        private Runnable duringClose;

        private LaneProbe(RuntimeException setupFailure,
                          Error setupError,
                          RuntimeException closeFailure) {
            this.setupFailure = setupFailure;
            this.setupError = setupError;
            this.closeFailure = closeFailure;
        }

        static LaneProbe failingSetup(RuntimeException setupFailure,
                                      RuntimeException closeFailure) {
            return new LaneProbe(setupFailure, null, closeFailure);
        }

        static LaneProbe failingSetup(Error setupError) {
            return new LaneProbe(null, setupError, null);
        }

        static LaneProbe open(RuntimeException closeFailure) {
            return new LaneProbe(null, null, closeFailure);
        }

        @Override
        public AprilTagSensor tagSensor() {
            if (setupError != null) {
                throw setupError;
            }
            if (setupFailure != null) {
                throw setupFailure;
            }
            return clock -> AprilTagDetections.none();
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            return CameraMountConfig.identity();
        }

        @Override
        public void close() {
            closeCount++;
            if (duringClose != null) {
                duringClose.run();
            }
            if (closeFailure != null) {
                throw closeFailure;
            }
        }
    }

    private static TesterContext context() {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        return new TesterContext(
                new HardwareMap(null, null),
                noOpTelemetry(),
                new Gamepad(),
                new Gamepad(),
                clock
        );
    }

    private static Telemetry noOpTelemetry() {
        InvocationHandler handler = (proxy, method, args) -> defaultValue(method.getReturnType());
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                handler
        );
    }

    private static Object defaultValue(Class<?> type) {
        if (type == boolean.class) return true;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0.0f;
        if (type == double.class) return 0.0;
        if (type == char.class) return '\0';
        return null;
    }

    private static void invokeEnsure(OwnerKind kind, TeleOpTester owner) {
        try {
            Method method = owner.getClass().getDeclaredMethod(kind.ensureMethod);
            method.setAccessible(true);
            method.invoke(owner);
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause();
            if (cause instanceof RuntimeException) {
                throw (RuntimeException) cause;
            }
            if (cause instanceof Error) {
                throw (Error) cause;
            }
            throw new AssertionError(cause);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }

    private static RuntimeException retainedFailure(TeleOpTester owner) {
        return (RuntimeException) field(owner, "visionFailure");
    }

    private static boolean booleanField(TeleOpTester owner, String name) {
        return (Boolean) field(owner, name);
    }

    private static Object field(TeleOpTester owner, String name) {
        try {
            Field field = owner.getClass().getDeclaredField(name);
            field.setAccessible(true);
            return field.get(owner);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }

    private static void setField(TeleOpTester owner, String name, Object value) {
        try {
            Field field = owner.getClass().getDeclaredField(name);
            field.setAccessible(true);
            field.set(owner, value);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }

    private static void setBooleanField(TeleOpTester owner, String name, boolean value) {
        setField(owner, name, value);
    }
}
