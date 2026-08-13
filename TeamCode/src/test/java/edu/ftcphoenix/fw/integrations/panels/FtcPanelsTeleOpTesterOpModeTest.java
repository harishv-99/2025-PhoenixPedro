package edu.ftcphoenix.fw.integrations.panels;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the Panels tester boundary without loading process-global Panels singletons. */
public final class FtcPanelsTeleOpTesterOpModeTest {

    @Test
    public void driverStationSourceUsesPhysicalIdentitiesAndNeverSamplesPanelsInput() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        backend.failIfInputIsRead = true;

        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.DRIVER_STATION,
                backend,
                driverTelemetry,
                tester);
        mode.gamepad1.a = true;
        mode.gamepad1.left_stick_x = 0.75f;
        mode.gamepad2.b = true;

        Gamepad physicalOne = mode.gamepad1;
        Gamepad physicalTwo = mode.gamepad2;
        mode.init();

        assertSame(physicalOne, tester.context.gamepad1);
        assertSame(physicalTwo, tester.context.gamepad2);
        assertTrue(tester.context.gamepad1.a);
        assertEquals(0.75f, tester.context.gamepad1.left_stick_x, 0.0f);
        assertTrue(tester.context.gamepad2.b);
        assertEquals(0, backend.clientCountCalls);
        assertEquals(0, backend.firstSnapshotCalls);
        assertEquals(0, backend.secondSnapshotCalls);

        mode.init_loop();
        mode.start();
        mode.loop();

        assertEquals(0, backend.clientCountCalls);
        assertEquals(0, backend.firstSnapshotCalls);
        assertEquals(0, backend.secondSnapshotCalls);
        mode.stop();
    }

    @Test
    public void exactOneClientRequirementRejectsDriverStationInput() {
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());

        RuntimeException thrown = expectRuntimeException(() -> new TestOpMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.DRIVER_STATION,
                FtcPanelsTeleOpTesterOpMode.PanelsClientRequirement.EXACTLY_ONE,
                backend,
                new RecordingTester()));

        assertTrue(thrown.getMessage().contains("EXACTLY_ONE"));
        assertTrue(thrown.getMessage().contains("InputSource.PANELS"));
        assertEquals(0, backend.clientCountCalls);
    }

    @Test
    public void panelsSourceUsesStableSyntheticIdentitiesAndSamplesBeforeEveryCallback() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                backend,
                driverTelemetry,
                tester);
        Gamepad physicalOne = mode.gamepad1;
        Gamepad physicalTwo = mode.gamepad2;
        physicalOne.left_stick_x = 0.91f;
        physicalTwo.left_stick_x = -0.91f;

        backend.firstSnapshot = gamepadWithLeftX(0.10f);
        backend.secondSnapshot = gamepadWithLeftX(-0.10f);
        mode.init();

        backend.firstSnapshot = gamepadWithLeftX(0.20f);
        backend.secondSnapshot = gamepadWithLeftX(-0.20f);
        physicalOne.left_stick_x = 0.92f;
        physicalTwo.left_stick_x = -0.92f;
        mode.init_loop();

        backend.firstSnapshot = gamepadWithLeftX(0.30f);
        backend.secondSnapshot = gamepadWithLeftX(-0.30f);
        physicalOne.left_stick_x = 0.93f;
        physicalTwo.left_stick_x = -0.93f;
        mode.start();

        backend.firstSnapshot = gamepadWithLeftX(0.40f);
        backend.secondSnapshot = gamepadWithLeftX(-0.40f);
        physicalOne.left_stick_x = 0.94f;
        physicalTwo.left_stick_x = -0.94f;
        mode.loop();

        assertEquals(Arrays.asList("init", "initLoop", "start", "loop"), tester.callbacks);
        assertFloatListEquals(
                Arrays.asList(0.10f, 0.20f, 0.30f, 0.40f),
                tester.firstLeftX);
        assertFloatListEquals(
                Arrays.asList(-0.10f, -0.20f, -0.30f, -0.40f),
                tester.secondLeftX);
        assertEquals(4, backend.firstSnapshotCalls);
        assertEquals(4, backend.secondSnapshotCalls);
        assertEquals(8, backend.clientCountCalls);

        Gamepad stableOne = tester.firstIdentities.get(0);
        Gamepad stableTwo = tester.secondIdentities.get(0);
        assertNotSame(physicalOne, stableOne);
        assertNotSame(physicalTwo, stableTwo);
        for (Gamepad identity : tester.firstIdentities) {
            assertSame(stableOne, identity);
        }
        for (Gamepad identity : tester.secondIdentities) {
            assertSame(stableTwo, identity);
        }
        mode.stop();
    }

    @Test
    public void defaultAtLeastOnePolicyAcceptsMultiplePanelsClients() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        backend.connectedClientCount = 2;
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                backend,
                driverTelemetry,
                tester);

        mode.init();

        assertEquals(1, tester.initCalls);
        assertEquals(2, backend.clientCountCalls);
        assertEquals(1, backend.firstSnapshotCalls);
        assertEquals(1, backend.secondSnapshotCalls);
        mode.stop();
    }

    @Test
    public void exactOnePolicyAcceptsExactlyOnePanelsClient() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                FtcPanelsTeleOpTesterOpMode.PanelsClientRequirement.EXACTLY_ONE,
                backend,
                driverTelemetry,
                tester);

        mode.init();

        assertEquals(1, tester.initCalls);
        assertEquals(2, backend.clientCountCalls);
        assertEquals(1, backend.firstSnapshotCalls);
        assertEquals(1, backend.secondSnapshotCalls);
        mode.stop();
    }

    @Test
    public void panelsOptionsAndShareSurviveStableGamepadCopyAsStartAndBack() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        Gamepad first = new Gamepad();
        first.options = true;
        first.share = true;
        assertFalse(first.start);
        assertFalse(first.back);
        backend.firstSnapshot = first;

        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                backend,
                driverTelemetry,
                tester);

        mode.init();

        assertTrue(tester.context.gamepad1.start);
        assertTrue(tester.context.gamepad1.options);
        assertTrue(tester.context.gamepad1.back);
        assertTrue(tester.context.gamepad1.share);
        mode.stop();
    }

    @Test
    public void readyFrameUpdatesDriverStationAndPanelsExactlyOnce() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.DRIVER_STATION,
                backend,
                driverTelemetry,
                tester);

        mode.init();

        assertEquals(1, driverTelemetry.updateCalls);
        assertEquals(1, panelsTelemetry.updateCalls);
        assertEquals(1, driverTelemetry.countLinesContaining("Ready: Recording tester"));
        assertEquals(1, panelsTelemetry.countLinesContaining("Ready: Recording tester"));
        mode.stop();
    }

    @Test
    public void activeTesterRowFrameMirrorsEveryOperationAndCommitsEachSinkOnce() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        RecordingTester tester = new RecordingTester();
        tester.emitRowFrameOnLoop = true;
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.DRIVER_STATION,
                backend,
                driverTelemetry,
                tester);
        mode.init();

        mode.start();
        mode.loop();

        assertEquals(2, driverTelemetry.updateCalls);
        assertEquals(2, panelsTelemetry.updateCalls);
        assertEquals(1, driverTelemetry.countLinesContaining("Power: 0.25"));
        assertEquals(1, panelsTelemetry.countLinesContaining("Power: 0.25"));
        assertEquals(1, driverTelemetry.countLinesContaining("Active tester frame"));
        assertEquals(1, panelsTelemetry.countLinesContaining("Active tester frame"));
        mode.stop();
    }

    @Test
    public void noPanelsClientPreventsInitWithoutStoppingUntouchedTester() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        backend.connectedClientCount = 0;
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                backend,
                driverTelemetry,
                tester);

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown.getMessage().contains("Panels tester input disconnected"));
        assertTrue(thrown.getMessage().contains("restart this tester OpMode"));
        assertFalse(thrown.getMessage().contains("FW: Testers (Panels)"));
        assertEquals(0, tester.initCalls);
        assertEquals(0, tester.stopCalls);
        assertEquals(1, backend.clientCountCalls);
        assertEquals(0, backend.firstSnapshotCalls);
        assertEquals(0, backend.secondSnapshotCalls);

        mode.init();
        mode.init_loop();
        mode.start();
        mode.loop();
        mode.stop();
        assertEquals(0, tester.stopCalls);
        assertEquals(1, backend.clientCountCalls);
    }

    @Test
    public void exactOnePolicyRejectsZeroOrTwoClientsBeforeTesterInit() {
        assertExactOneInitialClientCountRejected(0);
        assertExactOneInitialClientCountRejected(2);
    }

    private static void assertExactOneInitialClientCountRejected(int clientCount) {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        backend.connectedClientCount = clientCount;
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                FtcPanelsTeleOpTesterOpMode.PanelsClientRequirement.EXACTLY_ONE,
                backend,
                driverTelemetry,
                tester);

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown.getMessage().contains("requires exactly one connected Panels client"));
        assertTrue(thrown.getMessage().contains("found " + clientCount));
        assertTrue(thrown.getMessage().contains("restart this tester OpMode"));
        assertEquals(0, tester.initCalls);
        assertEquals(0, tester.stopCalls);
        assertEquals(1, backend.clientCountCalls);
        assertEquals(0, backend.firstSnapshotCalls);
        assertEquals(0, backend.secondSnapshotCalls);
        mode.stop();
        assertEquals(0, tester.stopCalls);
    }

    @Test
    public void exactOnePolicyRejectsClientCountThatChangesDuringInitialSnapshot() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        backend.connectedClientCountSequence = new int[]{1, 2};
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                FtcPanelsTeleOpTesterOpMode.PanelsClientRequirement.EXACTLY_ONE,
                backend,
                driverTelemetry,
                tester);

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown.getMessage().contains("found 2"));
        assertEquals(0, tester.initCalls);
        assertEquals(0, tester.stopCalls);
        assertEquals(2, backend.clientCountCalls);
        assertEquals(1, backend.firstSnapshotCalls);
        assertEquals(1, backend.secondSnapshotCalls);
    }

    @Test
    public void disconnectAfterInitStopsOnceAndPreventsLaterCallbacksOrSnapshots() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                backend,
                driverTelemetry,
                tester);
        mode.init();
        assertEquals(1, tester.initCalls);

        backend.connectedClientCount = 0;
        RuntimeException thrown = expectRuntimeException(mode::init_loop);

        assertTrue(thrown.getMessage().contains("Panels tester input disconnected"));
        assertEquals(1, tester.stopCalls);
        assertEquals(0, tester.initLoopCalls);
        int clientCallsAfterFailure = backend.clientCountCalls;
        int firstCallsAfterFailure = backend.firstSnapshotCalls;
        int secondCallsAfterFailure = backend.secondSnapshotCalls;

        backend.connectedClientCount = 1;
        mode.init_loop();
        mode.start();
        mode.loop();
        mode.stop();

        assertEquals(0, tester.initLoopCalls);
        assertEquals(0, tester.startCalls);
        assertEquals(0, tester.loopCalls);
        assertEquals(1, tester.stopCalls);
        assertEquals(clientCallsAfterFailure, backend.clientCountCalls);
        assertEquals(firstCallsAfterFailure, backend.firstSnapshotCalls);
        assertEquals(secondCallsAfterFailure, backend.secondSnapshotCalls);
    }

    @Test
    public void exactOnePolicyLaterZeroOrTwoClientsFailStopsExactlyOnce() {
        assertExactOneLaterClientCountFailStops(0);
        assertExactOneLaterClientCountFailStops(2);
    }

    private static void assertExactOneLaterClientCountFailStops(int clientCount) {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                FtcPanelsTeleOpTesterOpMode.PanelsClientRequirement.EXACTLY_ONE,
                backend,
                driverTelemetry,
                tester);
        mode.init();
        assertEquals(1, tester.initCalls);

        backend.connectedClientCount = clientCount;
        RuntimeException thrown = expectRuntimeException(mode::init_loop);

        assertTrue(thrown.getMessage().contains("requires exactly one connected Panels client"));
        assertTrue(thrown.getMessage().contains("found " + clientCount));
        assertEquals(1, tester.stopCalls);
        assertEquals(0, tester.initLoopCalls);
        int clientCallsAfterFailure = backend.clientCountCalls;
        int firstCallsAfterFailure = backend.firstSnapshotCalls;
        int secondCallsAfterFailure = backend.secondSnapshotCalls;

        backend.connectedClientCount = 1;
        mode.init_loop();
        mode.start();
        mode.loop();
        mode.stop();

        assertEquals(1, tester.stopCalls);
        assertEquals(0, tester.initLoopCalls);
        assertEquals(0, tester.startCalls);
        assertEquals(0, tester.loopCalls);
        assertEquals(clientCallsAfterFailure, backend.clientCountCalls);
        assertEquals(firstCallsAfterFailure, backend.firstSnapshotCalls);
        assertEquals(secondCallsAfterFailure, backend.secondSnapshotCalls);
    }

    @Test
    public void initialSnapshotFailureIsActionableAndLeavesUntouchedTesterInactive() {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        RuntimeException cause = new IllegalStateException("controlled snapshot failure");
        backend.firstSnapshotFailure = cause;
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                backend,
                driverTelemetry,
                tester);

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown.getMessage().contains("Panels tester input failed"));
        assertTrue(thrown.getMessage().contains("reconnect Panels"));
        assertSame(cause, thrown.getCause());
        assertEquals(0, tester.initCalls);
        assertEquals(0, tester.stopCalls);
        assertEquals(1, backend.firstSnapshotCalls);
        assertEquals(0, backend.secondSnapshotCalls);

        mode.stop();
        assertEquals(0, tester.stopCalls);
    }

    @Test
    public void nullPanelsSnapshotsFailClosed() {
        assertNullSnapshotFails(true);
        assertNullSnapshotFails(false);
    }

    private static void assertNullSnapshotFails(boolean firstIsNull) {
        RecordingTelemetry driverTelemetry = new RecordingTelemetry();
        RecordingTelemetry panelsTelemetry = new RecordingTelemetry();
        FakePanelsBackend backend = new FakePanelsBackend(panelsTelemetry.proxy());
        if (firstIsNull) {
            backend.firstSnapshot = null;
        } else {
            backend.secondSnapshot = null;
        }
        RecordingTester tester = new RecordingTester();
        TestOpMode mode = configuredMode(
                FtcPanelsTeleOpTesterOpMode.InputSource.PANELS,
                backend,
                driverTelemetry,
                tester);

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown.getMessage().contains("Panels tester input failed"));
        assertTrue(thrown.getCause() instanceof IllegalStateException);
        assertTrue(thrown.getCause().getMessage().contains("null virtual-gamepad snapshot"));
        assertEquals(0, tester.initCalls);
        assertEquals(0, tester.stopCalls);
        assertEquals(1, backend.firstSnapshotCalls);
        assertEquals(1, backend.secondSnapshotCalls);
        mode.stop();
        assertEquals(0, tester.stopCalls);
    }

    private static TestOpMode configuredMode(
            FtcPanelsTeleOpTesterOpMode.InputSource inputSource,
            FakePanelsBackend backend,
            RecordingTelemetry driverTelemetry,
            RecordingTester tester
    ) {
        return configuredMode(
                inputSource,
                FtcPanelsTeleOpTesterOpMode.PanelsClientRequirement.AT_LEAST_ONE,
                backend,
                driverTelemetry,
                tester);
    }

    private static TestOpMode configuredMode(
            FtcPanelsTeleOpTesterOpMode.InputSource inputSource,
            FtcPanelsTeleOpTesterOpMode.PanelsClientRequirement clientRequirement,
            FakePanelsBackend backend,
            RecordingTelemetry driverTelemetry,
            RecordingTester tester
    ) {
        TestOpMode mode = new TestOpMode(inputSource, clientRequirement, backend, tester);
        mode.telemetry = driverTelemetry.proxy();
        mode.hardwareMap = new HardwareMap(null, null);
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        return mode;
    }

    private static Gamepad gamepadWithLeftX(float value) {
        Gamepad gamepad = new Gamepad();
        gamepad.left_stick_x = value;
        return gamepad;
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

    private static void assertFloatListEquals(List<Float> expected, List<Float> actual) {
        assertEquals(expected.size(), actual.size());
        for (int i = 0; i < expected.size(); i++) {
            assertEquals(expected.get(i), actual.get(i), 0.0f);
        }
    }

    private static final class TestOpMode extends FtcPanelsTeleOpTesterOpMode {
        private final TeleOpTester tester;
        private double runtimeSec;

        private TestOpMode(
                InputSource inputSource,
                PanelsClientRequirement clientRequirement,
                PanelsBackend backend,
                TeleOpTester tester
        ) {
            super(inputSource, clientRequirement, backend);
            this.tester = tester;
        }

        @Override
        protected TeleOpTester createTester() {
            return tester;
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final class FakePanelsBackend implements
            FtcPanelsTeleOpTesterOpMode.PanelsBackend {
        private final Telemetry telemetry;
        private int connectedClientCount = 1;
        private int[] connectedClientCountSequence;
        private int connectedClientCountIndex;
        private Gamepad firstSnapshot = new Gamepad();
        private Gamepad secondSnapshot = new Gamepad();
        private RuntimeException firstSnapshotFailure;
        private RuntimeException secondSnapshotFailure;
        private boolean failIfInputIsRead;
        private int clientCountCalls;
        private int firstSnapshotCalls;
        private int secondSnapshotCalls;

        private FakePanelsBackend(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public Telemetry telemetry() {
            return telemetry;
        }

        @Override
        public int connectedClientCount() {
            clientCountCalls++;
            failIfInputRead("connectedClientCount");
            if (connectedClientCountSequence != null
                    && connectedClientCountIndex < connectedClientCountSequence.length) {
                return connectedClientCountSequence[connectedClientCountIndex++];
            }
            return connectedClientCount;
        }

        @Override
        public Gamepad firstGamepadSnapshot() {
            firstSnapshotCalls++;
            failIfInputRead("firstGamepadSnapshot");
            if (firstSnapshotFailure != null) {
                throw firstSnapshotFailure;
            }
            return firstSnapshot;
        }

        @Override
        public Gamepad secondGamepadSnapshot() {
            secondSnapshotCalls++;
            failIfInputRead("secondGamepadSnapshot");
            if (secondSnapshotFailure != null) {
                throw secondSnapshotFailure;
            }
            return secondSnapshot;
        }

        private void failIfInputRead(String operation) {
            if (failIfInputIsRead) {
                throw new AssertionError("Driver Station source called " + operation);
            }
        }
    }

    private static final class RecordingTester implements TeleOpTester {
        private TesterContext context;
        private final List<String> callbacks = new ArrayList<String>();
        private final List<Float> firstLeftX = new ArrayList<Float>();
        private final List<Float> secondLeftX = new ArrayList<Float>();
        private final List<Gamepad> firstIdentities = new ArrayList<Gamepad>();
        private final List<Gamepad> secondIdentities = new ArrayList<Gamepad>();
        private int initCalls;
        private int initLoopCalls;
        private int startCalls;
        private int loopCalls;
        private int stopCalls;
        private boolean emitRowFrameOnLoop;

        @Override
        public String name() {
            return "Recording tester";
        }

        @Override
        public void init(TesterContext context) {
            initCalls++;
            this.context = context;
            record("init");
        }

        @Override
        public void initLoop(double dtSec) {
            initLoopCalls++;
            record("initLoop");
        }

        @Override
        public void start() {
            startCalls++;
            record("start");
        }

        @Override
        public void loop(double dtSec) {
            loopCalls++;
            record("loop");
            if (emitRowFrameOnLoop) {
                context.telemetry.clearAll();
                context.telemetry.addData("Power", 0.25);
                context.telemetry.addLine("Active tester frame");
                context.telemetry.update();
            }
        }

        @Override
        public void stop() {
            stopCalls++;
        }

        private void record(String callback) {
            callbacks.add(callback);
            firstLeftX.add(context.gamepad1.left_stick_x);
            secondLeftX.add(context.gamepad2.left_stick_x);
            firstIdentities.add(context.gamepad1);
            secondIdentities.add(context.gamepad2);
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> lines = new ArrayList<String>();
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this);
        private int updateCalls;

        private Telemetry proxy() {
            return proxy;
        }

        private int countLinesContaining(String text) {
            int count = 0;
            for (String line : lines) {
                if (line.contains(text)) {
                    count++;
                }
            }
            return count;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method.getName(), args);
            }
            if ("addLine".equals(method.getName())) {
                lines.add(args == null || args.length == 0 ? "" : String.valueOf(args[0]));
            } else if ("addData".equals(method.getName()) && args != null && args.length >= 2) {
                lines.add(String.valueOf(args[0]) + ": " + String.valueOf(args[1]));
            } else if ("update".equals(method.getName())) {
                updateCalls++;
            } else if ("clear".equals(method.getName())
                    || "clearAll".equals(method.getName())) {
                lines.clear();
            }
            return defaultValue(method.getReturnType());
        }

        private static Object objectMethod(Object proxy, String name, Object[] args) {
            if ("toString".equals(name)) {
                return "RecordingTelemetry";
            }
            if ("hashCode".equals(name)) {
                return System.identityHashCode(proxy);
            }
            if ("equals".equals(name)) {
                return proxy == (args == null ? null : args[0]);
            }
            return null;
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
