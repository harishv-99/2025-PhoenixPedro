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
import edu.ftcphoenix.fw.ftc.ui.HardwareNamePicker;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
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
    public void factoryFailureWithSuppressedRollbackBlocksReplacement() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException primary = new IllegalStateException(kind + " factory failed");
            RuntimeException rollback = new IllegalArgumentException(kind + " rollback failed");
            primary.addSuppressed(rollback);
            OpenProbe opener = OpenProbe.failingOpen(primary);
            TeleOpTester owner = kind.create(opener);

            owner.init(context());

            assertSame(kind.toString(), primary, retainedFailure(owner));
            assertTrue(kind + " must classify unpublished cleanup as uncertain",
                    booleanField(owner, "visionCleanupFailed"));
            assertTrue(kind + " must consume BACK while replacement is blocked",
                    owner.onBackPressed());

            invokeEnsure(kind, owner);
            owner.stop();
            owner.stop();
            assertEquals(kind + " must never open over uncertain constructor cleanup",
                    1, opener.openCount);
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
    public void pendingOwnerBlocksReplacementAndBackStillClosesIt() {
        for (OwnerKind kind : OwnerKind.values()) {
            LaneProbe pending = LaneProbe.open(null);
            pending.readiness = VisionReadiness.notReady("camera is still opening");
            OpenProbe opener;
            TeleOpTester owner;
            if (kind == OwnerKind.PINPOINT_FUSION) {
                // This tester also owns real Pinpoint hardware, which is outside this pure-JVM
                // fixture. Initialize its context through the existing post-open failure seam,
                // then install the retained pending camera owner and clear that unrelated failure.
                opener = new OpenProbe(LaneProbe.failingSetup(
                        new IllegalStateException("fixture stops before Pinpoint lookup"), null));
                owner = kind.create(opener);
                owner.init(context());
                setField(owner, kind.laneField, pending);
                setBooleanField(owner, kind.readyField, false);
                setField(owner, "visionFailure", null);
                setField(owner, "initError", null);
            } else {
                opener = new OpenProbe(pending);
                owner = kind.create(opener);
                owner.init(context());
            }

            invokeEnsure(kind, owner);

            assertEquals(kind + " must not open over a pending owner", 1, opener.openCount);
            assertFalse(kind + " must remain pending", booleanField(owner, kind.readyField));

            pending.readiness = VisionReadiness.ready();
            invokeEnsure(kind, owner);
            assertTrue(kind + " must become ready without reopening",
                    booleanField(owner, kind.readyField));
            assertEquals(kind + " readiness transition must retain one owner", 1, opener.openCount);

            assertTrue(kind + " BACK must close a retained pending owner", owner.onBackPressed());
            assertEquals(kind.toString(), 1, pending.closeCount);
            assertFalse(kind + " returns to the picker after confirmed close", owner.onBackPressed());
        }
    }

    @Test
    public void dynamicReadinessFailureClosesAndDetachesBeforeAllowingFreshSelection() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException readinessFailure =
                    new IllegalStateException(kind + " readiness failed");
            LaneProbe active = LaneProbe.open(null);
            active.readiness = VisionReadiness.notReady("camera is still opening");
            OpenProbe opener;
            TeleOpTester owner;
            if (kind == OwnerKind.PINPOINT_FUSION) {
                opener = new OpenProbe(LaneProbe.failingSetup(
                        new IllegalStateException("fixture stops before Pinpoint lookup"), null));
                owner = kind.create(opener);
                owner.init(context());
                setField(owner, kind.laneField, active);
                setBooleanField(owner, kind.readyField, false);
                setField(owner, "visionFailure", null);
                setField(owner, "initError", null);
            } else {
                opener = new OpenProbe(active);
                owner = kind.create(opener);
                owner.init(context());
            }

            active.readinessFailure = readinessFailure;
            invokeEnsure(kind, owner);

            assertEquals(kind.toString(), 1, active.closeCount);
            assertSame(kind.toString(), readinessFailure, retainedFailure(owner));
            assertSame(kind + " must detach the failed owner", null,
                    field(owner, kind.laneField));
            assertFalse(kind + " confirmed cleanup must permit a fresh selection",
                    booleanField(owner, "visionCleanupFailed"));
            assertFalse(kind + " must leave the closing transition",
                    booleanField(owner, kind.closingField));
            owner.stop();
            owner.stop();
            assertEquals(kind + " detached owner must never be closed twice", 1,
                    active.closeCount);
        }
    }

    @Test
    public void pinpointPodOffsetAssistBlocksReplacementWhenFailedOwnerCannotClose() {
        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        cfg.enableAprilTagAssist = true;
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(cfg);
        RuntimeException readinessFailure = new IllegalStateException("readiness failed");
        RuntimeException closeFailure = new IllegalArgumentException("close failed");
        LaneProbe lane = LaneProbe.open(closeFailure);
        setField(owner, "visionLane", lane);
        setField(owner, "selectedVisionDeviceName", "vision");

        invokePrivate(
                owner,
                "handleVisionFailure",
                new Class<?>[]{RuntimeException.class, String.class, boolean.class},
                readinessFailure,
                "Vision readiness failed",
                false
        );

        assertEquals(1, lane.closeCount);
        assertSame(null, field(owner, "visionLane"));
        assertTrue(booleanField(owner, "visionCleanupFailed"));
        VisionReadiness readiness = (VisionReadiness) field(owner, "visionReadiness");
        assertTrue(readiness.reason(), readiness.reason().contains("restart this OpMode"));
        assertEquals(1, readinessFailure.getSuppressed().length);
        assertSame(closeFailure, readinessFailure.getSuppressed()[0]);
        assertSame(readinessFailure, field(owner, "visionFailure"));
        assertTrue(readiness.reason(), readiness.reason().contains(
                "IllegalStateException: readiness failed"));
        assertTrue(readiness.reason(), readiness.reason().contains(
                "cleanup also failed: IllegalArgumentException: close failed"));

        setField(owner, "selectedVisionDeviceName", "replacement");
        invokePrivate(
                owner,
                "ensureAprilTagAssistReady",
                new Class<?>[]{boolean.class},
                true
        );
        assertEquals("cleanup-failed owner must never be closed or replaced again",
                1, lane.closeCount);
    }

    @Test
    public void pinpointPodOffsetNullReadinessClosesAndOffersInitRetryWithCause() {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator();
        LaneProbe lane = LaneProbe.open(null);
        lane.readiness = null;
        TesterContext ctx = context();

        setField(owner, "ctx", ctx);
        setField(owner, "visionLane", lane);
        setField(owner, "selectedVisionDeviceName", "vision");
        setField(owner, "visionPicker", new HardwareNamePicker(
                ctx.hw,
                HardwareDevice.class,
                "Select Vision Device"
        ));

        invokePrivate(
                owner,
                "refreshAprilTagVisionReadiness",
                new Class<?>[]{boolean.class},
                true
        );

        assertEquals(1, lane.closeCount);
        assertSame(null, field(owner, "visionLane"));
        assertFalse(booleanField(owner, "visionCleanupFailed"));
        RuntimeException failure = (RuntimeException) field(owner, "visionFailure");
        assertTrue(failure.getMessage(), failure.getMessage().contains("null readiness"));
        VisionReadiness readiness = (VisionReadiness) field(owner, "visionReadiness");
        assertTrue(readiness.reason(), readiness.reason().contains(
                "IllegalStateException: vision lane returned a null readiness result"));
        assertTrue(readiness.reason(), readiness.reason().contains(
                "select the vision device again"));
    }

    @Test
    public void pinpointPodOffsetActiveReadinessFailureRequiresReopenInsteadOfHiddenPicker() {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator();
        RuntimeException readinessFailure = new IllegalStateException("USB disconnected");
        LaneProbe lane = LaneProbe.open(null);
        lane.readinessFailure = readinessFailure;
        TesterContext ctx = context();

        setField(owner, "ctx", ctx);
        setField(owner, "visionLane", lane);
        setField(owner, "selectedVisionDeviceName", "vision");
        setField(owner, "visionPicker", new HardwareNamePicker(
                ctx.hw,
                HardwareDevice.class,
                "Select Vision Device"
        ));

        invokePrivate(
                owner,
                "refreshAprilTagVisionReadiness",
                new Class<?>[]{boolean.class},
                false
        );

        assertEquals(1, lane.closeCount);
        assertSame(readinessFailure, field(owner, "visionFailure"));
        VisionReadiness readiness = (VisionReadiness) field(owner, "visionReadiness");
        assertTrue(readiness.reason(), readiness.reason().contains(
                "IllegalStateException: USB disconnected"));
        assertTrue(readiness.reason(), readiness.reason().contains(
                "press BACK and reopen this tester"));
        assertFalse(readiness.reason(), readiness.reason().contains(
                "select the vision device again"));
    }

    @Test
    public void pinpointPodOffsetFactoryFailureWithSuppressedRollbackBlocksReplacement() {
        RuntimeException primary = new IllegalStateException("factory failed");
        RuntimeException rollback = new IllegalArgumentException("rollback failed");
        primary.addSuppressed(rollback);
        final int[] openCalls = {0};

        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        cfg.enableAprilTagAssist = true;
        cfg.cameraMount = CameraMountConfig.of(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        cfg.visionLaneFactoryBuilder = ignored -> hardwareMap -> {
            openCalls[0]++;
            throw primary;
        };
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(cfg);
        setField(owner, "ctx", context());
        setField(owner, "selectedVisionDeviceName", "vision");

        invokePrivate(
                owner,
                "ensureAprilTagAssistReady",
                new Class<?>[]{boolean.class},
                true
        );

        assertSame(primary, field(owner, "visionFailure"));
        assertTrue(booleanField(owner, "visionCleanupFailed"));
        VisionReadiness readiness = (VisionReadiness) field(owner, "visionReadiness");
        assertTrue(readiness.reason(), readiness.reason().contains("restart this OpMode"));

        setField(owner, "selectedVisionDeviceName", "replacement");
        invokePrivate(
                owner,
                "ensureAprilTagAssistReady",
                new Class<?>[]{boolean.class},
                true
        );
        assertEquals(1, openCalls[0]);
    }

    @Test
    public void pinpointPodOffsetFinalStopDetachesBeforeExactOnceClose() {
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator();
        RuntimeException cleanup = new IllegalStateException("final close failed");
        LaneProbe lane = LaneProbe.open(cleanup);
        setField(owner, "visionLane", lane);
        lane.duringClose = owner::stop;

        try {
            owner.stop();
            fail("Expected final close failure");
        } catch (RuntimeException actual) {
            assertSame(cleanup, actual);
        }

        owner.stop();
        assertEquals(1, lane.closeCount);
        assertSame(null, field(owner, "visionLane"));
        assertTrue(booleanField(owner, "visionTerminalRequested"));
        assertTrue(booleanField(owner, "visionCleanupFailed"));
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
        private RuntimeException openFailure;
        private int openCount;

        OpenProbe(LaneProbe... lanes) {
            this.lanes = new ArrayDeque<>(Arrays.asList(lanes));
        }

        static OpenProbe failingOpen(RuntimeException openFailure) {
            OpenProbe probe = new OpenProbe();
            probe.openFailure = openFailure;
            return probe;
        }

        AprilTagVisionLaneFactory factory() {
            return hardwareMap -> {
                openCount++;
                if (openFailure != null) {
                    throw openFailure;
                }
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
        private VisionReadiness readiness = VisionReadiness.ready();
        private RuntimeException readinessFailure;

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
        public VisionReadiness readiness(LoopClock clock) {
            if (readinessFailure != null) {
                throw readinessFailure;
            }
            return readiness;
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

    private static void invokePrivate(
            Object owner,
            String methodName,
            Class<?>[] parameterTypes,
            Object... args
    ) {
        try {
            Method method = owner.getClass().getDeclaredMethod(methodName, parameterTypes);
            method.setAccessible(true);
            method.invoke(owner, args);
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
            Field field = findField(owner.getClass(), name);
            field.setAccessible(true);
            return field.get(owner);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }

    private static void setField(TeleOpTester owner, String name, Object value) {
        try {
            Field field = findField(owner.getClass(), name);
            field.setAccessible(true);
            field.set(owner, value);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(e);
        }
    }

    private static void setBooleanField(TeleOpTester owner, String name, boolean value) {
        setField(owner, name, value);
    }

    private static Field findField(Class<?> type, String name) throws NoSuchFieldException {
        Class<?> current = type;
        while (current != null) {
            try {
                return current.getDeclaredField(name);
            } catch (NoSuchFieldException ignored) {
                current = current.getSuperclass();
            }
        }
        throw new NoSuchFieldException(name);
    }
}
