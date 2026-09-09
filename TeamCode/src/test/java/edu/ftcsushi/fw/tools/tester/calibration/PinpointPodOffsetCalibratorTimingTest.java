package edu.ftcsushi.fw.tools.tester.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.MecanumDrivebase;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.input.Gamepads;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.ftc.vision.AprilTagVision;
import edu.ftcsushi.fw.ftc.vision.OwnedAprilTagCamera;
import edu.ftcsushi.fw.ftc.vision.VisionReadiness;
import edu.ftcsushi.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.tools.tester.TesterContext;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;
import edu.ftcsushi.fw.tools.tester.TesterSuite;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Serviced-loop CAL-05 evidence, not a physical braking or sensor-reliability claim.
 *
 * <p>The real calibrator, phase transitions, clock, Pinpoint predictor, and mecanum mixer remain
 * active. Reflection supplies already-owned resources in place of FTC hardware acquisition and
 * uses the predictor's existing package-private device seam for finite scripted observations.
 * Pending flags represent binding-edge requests; direct private Abort invocation represents the
 * existing binding callback, without recreating its registration as purported production proof.
 * Camera acquisition is replaced with scripted timestamped detections; the real AprilTag solver
 * and bounded raw-odometry history retain capture-time matching. These observations do not prove
 * physical timing accuracy, braking, or pod geometry.
 */
public final class PinpointPodOffsetCalibratorTimingTest {

    private static final double LIMIT = 10.0;
    private static final String[] POWERED_PHASES = {
            "SEARCH_TAG_START", "ROTATING", "SEARCH_TAG_END"
    };

    @Test
    public void frozenReadyHeadingExpiresEachPoweredPhaseAtItsOwnExactDeadline() throws Exception {
        for (String phase : POWERED_PHASES) {
            Fixture fixture = enteredPhase(phase);
            double enteredAt = fixture.clock.nowSec();
            fixture.loopAt(enteredAt + LIMIT - 0.001);
            assertEquals(phase, fixture.phase());
            fixture.assertPowered();
            assertTrue(fixture.predictor.getKinematicSnapshot().hasUsableKinematics());

            int polls = fixture.device.polls;
            int commands = fixture.commands.size();
            fixture.loopAt(enteredAt + LIMIT);

            fixture.assertRejected();
            assertTrue(((String) field(fixture.owner, "lastAttemptFailure")).startsWith(phase + " "));
            assertEquals("deadline must precede the next Pinpoint poll", polls,
                    fixture.device.polls);
            fixture.assertOnlyZeroCommandsSince(commands);
            fixture.loopAt(enteredAt + LIMIT + 1.0);
            fixture.assertRejected();
            fixture.assertOnlyZeroCommandsSince(commands);
        }
    }

    @Test
    public void firstServicedLoopAfterDeadlineRejectsWithoutAnotherPoweredWrite() throws Exception {
        for (String phase : POWERED_PHASES) {
            Fixture fixture = enteredPhase(phase);
            double enteredAt = fixture.clock.nowSec();
            fixture.owner.loop(0.0);
            int commands = fixture.commands.size();
            int polls = fixture.device.polls;

            fixture.loopAt(enteredAt + LIMIT + 50.0);

            fixture.assertRejected();
            fixture.assertOnlyZeroCommandsSince(commands);
            assertEquals(polls, fixture.device.polls);
        }
    }

    @Test
    public void phaseMayExpireBeforeItsFirstRunWithoutInventingAMinimumPoweredPulse()
            throws Exception {
        Fixture fixture = new Fixture(false, true);
        invoke(fixture.owner, "requestStartSample", new Class<?>[]{boolean.class}, true);
        assertEquals("ROTATING", fixture.phase());
        fixture.assertOnlyZeroCommandsSince(0);
        int polls = fixture.device.polls;

        fixture.loopAt(LIMIT + 1.0);

        fixture.assertRejected();
        fixture.assertOnlyZeroCommandsSince(0);
        assertEquals(polls, fixture.device.polls);
    }

    @Test
    public void configuredDurationControlsTheDeadlineInsteadOfTheSoftwareDefault()
            throws Exception {
        Fixture fixture = new Fixture(false, true,
                config -> config.automaticPhaseTimeoutSec = 2.5);
        fixture.queueAuto();
        fixture.owner.loop(0.0);
        fixture.loopAt(2.499);
        assertEquals("ROTATING", fixture.phase());
        fixture.assertPowered();

        fixture.loopAt(2.5);

        fixture.assertRejected();
        assertTrue(((String) field(fixture.owner, "lastAttemptFailure"))
                .contains("2.50 / 2.50 s"));
    }

    @Test
    public void manualSampleStartSearchIsPoweredAndTimedDespiteAutoSampleBeingFalse()
            throws Exception {
        Fixture fixture = new Fixture(true, true);
        fixture.queuePrimary();
        fixture.owner.loop(0.0);
        assertEquals("SEARCH_TAG_START", fixture.phase());
        assertFalse((Boolean) field(fixture.owner, "autoSample"));
        fixture.assertPowered();

        fixture.loopAt(LIMIT);

        fixture.assertRejected();
    }

    @Test
    public void expiryBeatsQueuedSkipFinishRestartResetAndNewSuccessEvidence() throws Exception {
        for (String phase : POWERED_PHASES) {
            Fixture fixture = enteredPhase(phase);
            double enteredAt = fixture.clock.nowSec();
            fixture.queuePrimary();
            fixture.queueAuto();
            setField(fixture.owner, "resetRequested", true);
            fixture.device.pose = sdkPose(Math.PI);
            fixture.frameAt(fixture.clock.nowTimestamp(), Pose2d.zero());
            setField(fixture.owner, "tagStableFrames", 3);
            setField(fixture.owner, "lastRecommendedStrafePodOffsetForwardInches", 12.0);
            setField(fixture.owner, "lastRecommendedForwardPodOffsetLeftInches", 13.0);
            int rebases = fixture.device.rebases;
            int commands = fixture.commands.size();

            fixture.loopAt(enteredAt + LIMIT);

            fixture.assertRejected();
            assertEquals("expiry must discard X/A before either can rebase", rebases,
                    fixture.device.rebases);
            assertFalse((Boolean) field(fixture.owner, "primaryActionRequested"));
            assertFalse((Boolean) field(fixture.owner, "autoStartRequested"));
            assertFalse((Boolean) field(fixture.owner, "resetRequested"));
            fixture.assertOnlyZeroCommandsSince(commands);
        }
    }

    @Test
    public void largePreEntryIntervalAndRepeatedSameCycleLoopsDoNotSpendTheNewBudget()
            throws Exception {
        Fixture fixture = new Fixture(false, true);
        fixture.clock.update(500.0);
        fixture.queueAuto();
        fixture.owner.loop(fixture.clock.dtSec());
        assertEquals("ROTATING", fixture.phase());
        fixture.assertPowered();

        for (int i = 0; i < 5; i++) {
            fixture.owner.loop(500.0);
            assertEquals("ROTATING", fixture.phase());
        }
        fixture.loopAt(509.999);
        assertEquals("ROTATING", fixture.phase());
        fixture.loopAt(510.0);
        fixture.assertRejected();
    }

    @Test
    public void skippingStartSearchBeginsFreshAutomaticRotationBudget() throws Exception {
        Fixture fixture = new Fixture(true, true);
        fixture.queueAuto();
        fixture.owner.loop(0.0);
        fixture.queuePrimary();
        fixture.loopAt(9.0);
        assertEquals("ROTATING", fixture.phase());

        fixture.loopAt(18.999);
        assertEquals("ROTATING", fixture.phase());
        fixture.assertPowered();
        fixture.loopAt(19.0);
        fixture.assertRejected();
    }

    @Test
    public void successfulRotationBeginsFreshEndSearchBudget() throws Exception {
        Fixture fixture = new Fixture(true, true);
        fixture.startAutoWithTag();
        fixture.device.pose = sdkPose(Math.PI);
        fixture.loopAt(9.0);
        assertEquals("SEARCH_TAG_END", fixture.phase());

        fixture.loopAt(18.999);
        assertEquals("SEARCH_TAG_END", fixture.phase());
        fixture.assertPowered();
        fixture.loopAt(19.0);
        fixture.assertRejected();
    }

    @Test
    public void timeoutInhibitsReplayedSameCycleIntentButAllowsFreshLaterAttempt() throws Exception {
        Fixture fixture = enteredPhase("ROTATING");
        fixture.loopAt(LIMIT);
        fixture.assertRejected();
        int commands = fixture.commands.size();

        fixture.queuePrimary();
        fixture.queueAuto();
        setField(fixture.owner, "resetRequested", true);
        fixture.owner.loop(1000.0);
        fixture.assertRejected();
        fixture.assertOnlyZeroCommandsSince(commands);

        fixture.clock.update(LIMIT + 1.0);
        fixture.queueAuto();
        fixture.owner.loop(1.0);
        assertEquals("ROTATING", fixture.phase());
        fixture.assertPowered();
        fixture.loopAt(2.0 * LIMIT + 0.999);
        assertEquals("ROTATING", fixture.phase());
        fixture.loopAt(2.0 * LIMIT + 1.0);
        fixture.assertRejected();
    }

    @Test
    public void abortWinsOverQueuedButtonsAndResetCannotRemoveSameCycleInhibition()
            throws Exception {
        Fixture fixture = enteredPhase("ROTATING");
        fixture.queuePrimary();
        fixture.queueAuto();
        setField(fixture.owner, "resetRequested", true);
        invoke(fixture.owner, "abortSample");
        int commands = fixture.commands.size();
        fixture.owner.loop(0.0);
        assertEquals("IDLE", fixture.phase());
        fixture.assertOnlyZeroCommandsSince(commands);

        fixture.queueAuto();
        setField(fixture.owner, "resetRequested", true);
        fixture.owner.loop(0.0);
        assertEquals("IDLE", fixture.phase());
        fixture.assertOnlyZeroCommandsSince(commands);

        fixture.clock.update(1.0);
        fixture.queueAuto();
        fixture.owner.loop(1.0);
        assertEquals("ROTATING", fixture.phase());
        fixture.assertPowered();
        fixture.loopAt(11.0);
        fixture.assertRejected();
    }

    @Test
    public void resetEpochNonFiniteTimeAndUnavailableTimestampFailClosedBeforePolling()
            throws Exception {
        for (int invalidCase = 0; invalidCase < 3; invalidCase++) {
            Fixture fixture = enteredPhase("ROTATING");
            int polls = fixture.device.polls;
            int commands = fixture.commands.size();
            if (invalidCase == 0) {
                fixture.clock.reset(fixture.clock.nowSec());
            } else if (invalidCase == 1) {
                fixture.clock.update(Double.NaN);
            } else {
                setField(fixture.owner, "automaticPhaseStartedAt", LoopTimestamp.unavailable());
            }

            fixture.owner.loop(0.0);

            fixture.assertRejected();
            assertTrue(((String) field(fixture.owner, "lastAttemptFailure"))
                    .startsWith("ROTATING has invalid elapsed time"));
            assertEquals(polls, fixture.device.polls);
            fixture.assertOnlyZeroCommandsSince(commands);
        }
    }

    @Test
    public void manualStickRotationAndRecenterRemainUntimed() throws Exception {
        Fixture manual = new Fixture(false, true);
        manual.gamepad.right_stick_x = 0.5f;
        manual.queuePrimary();
        manual.owner.loop(0.0);
        assertEquals("ROTATING", manual.phase());
        assertFalse((Boolean) field(manual.owner, "autoSample"));
        manual.loopAt(100.0);
        assertEquals("ROTATING", manual.phase());
        manual.assertPowered();

        manual.queuePrimary();
        manual.loopAt(101.0);
        assertEquals("POST_RECENTER", manual.phase());
        manual.gamepad.left_stick_y = 0.5f;
        manual.loopAt(201.0);
        assertEquals("POST_RECENTER", manual.phase());
        manual.assertPowered();
    }

    @Test
    public void unpoweredManualWorkflowRemainsUsableWithoutTiming() throws Exception {
        Fixture fixture = new Fixture(false, false);
        fixture.queueAuto();
        fixture.owner.loop(0.0);
        assertEquals("IDLE", fixture.phase());
        fixture.queuePrimary();
        fixture.loopAt(1.0);
        assertEquals("ROTATING", fixture.phase());
        fixture.loopAt(1000.0);
        assertEquals("ROTATING", fixture.phase());
        assertTrue(fixture.commands.isEmpty());
    }

    @Test
    public void normalAngleCompletionClearsTimingAndPreservesManualRecenter() throws Exception {
        Fixture fixture = enteredPhase("ROTATING");
        fixture.device.pose = sdkPose(Math.PI);
        fixture.loopAt(1.0);
        assertEquals("POST_RECENTER", fixture.phase());
        fixture.assertStopped();
        fixture.loopAt(100.0);
        assertEquals("POST_RECENTER", fixture.phase());
        fixture.queuePrimary();
        fixture.loopAt(101.0);
        assertEquals("IDLE", fixture.phase());
        assertNotNull(field(fixture.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        assertNotNull(field(fixture.owner, "lastRecommendedForwardPodOffsetLeftInches"));
    }

    @Test
    public void startSearchAngularLimitDiscardsAssistedAttemptBeforeTimeout()
            throws Exception {
        Fixture fixture = new Fixture(true, true, config -> config.tagSearchMaxTurnRad = 1.0);
        fixture.queueAuto();
        fixture.owner.loop(0.0);
        fixture.device.pose = sdkPose(1.1);

        fixture.loopAt(1.0);

        assertEquals("IDLE", fixture.phase());
        assertNotNull(field(fixture.owner, "lastAttemptFailure"));
        assertNull(field(fixture.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        fixture.assertStopped();
        fixture.loopAt(2.0);
        fixture.assertStopped();
        assertEquals("IDLE", fixture.phase());
    }

    @Test
    public void endSearchAngularLimitDiscardsInsteadOfFallingBackToRecenter() throws Exception {
        Fixture fixture = new Fixture(true, true,
                config -> config.tagEndSearchMaxExtraTurnRad = 1.0);
        fixture.startAutoWithTag();
        fixture.device.pose = sdkPose(Math.PI);
        fixture.loopAt(1.0);
        assertEquals("SEARCH_TAG_END", fixture.phase());
        fixture.device.pose = sdkPose(Math.PI + 1.1);

        fixture.loopAt(2.0);

        assertEquals("IDLE", fixture.phase());
        assertNotNull(field(fixture.owner, "lastAttemptFailure"));
        assertNull(field(fixture.owner, "lastRecommendedStrafePodOffsetForwardInches"));
        fixture.assertStopped();
        fixture.loopAt(100.0);
        assertEquals("IDLE", fixture.phase());
        assertNotNull(field(fixture.owner, "lastAttemptFailure"));
    }

    @Test
    public void readinessLossAbortsAndExplicitResetDoesNotInheritOldPhaseTime() throws Exception {
        Fixture fixture = enteredPhase("ROTATING");
        fixture.device.status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        fixture.loopAt(1.0);
        assertEquals("IDLE", fixture.phase());
        fixture.assertStopped();
        fixture.device.status = GoBildaPinpointDriver.DeviceStatus.READY;
        fixture.loopAt(2.0);
        fixture.queueAuto();
        fixture.loopAt(3.0);
        assertEquals("ROTATING", fixture.phase());
        setField(fixture.owner, "resetRequested", true);
        fixture.loopAt(4.0);
        assertEquals("IDLE", fixture.phase());
        fixture.queueAuto();
        fixture.loopAt(5.0);
        fixture.loopAt(14.999);
        assertEquals("ROTATING", fixture.phase());
        fixture.loopAt(15.0);
        fixture.assertRejected();
    }

    @Test
    public void startClearsPendingIntentAndStopCannotLeaveAnActivePhase() throws Exception {
        Fixture fixture = enteredPhase("ROTATING");
        fixture.queuePrimary();
        fixture.queueAuto();
        fixture.owner.start();
        assertEquals("IDLE", fixture.phase());
        fixture.assertStopped();
        fixture.loopAt(100.0);
        assertEquals("IDLE", fixture.phase());
        fixture.queueAuto();
        fixture.loopAt(101.0);
        assertEquals("ROTATING", fixture.phase());

        fixture.owner.stop();
        assertEquals("IDLE", fixture.phase());
        assertFalse((Boolean) field(fixture.owner, "started"));
        fixture.assertStopped();
        int commands = fixture.commands.size();
        fixture.owner.stop();
        assertEquals(commands, fixture.commands.size());
        for (RecordingOutput output : fixture.outputs) assertEquals(1, output.stops);
    }

    @Test
    public void backThroughRealTesterSuiteStopsTheActivePoweredSearchOnce() throws Exception {
        Fixture fixture = enteredPhase("SEARCH_TAG_START");
        fixture.assertPowered();
        CountingCamera camera = new CountingCamera();
        setField(fixture.owner, "visionLane", camera.owned);
        TesterSuite suite = new TesterSuite();
        suite.init((TesterContext) field(fixture.owner, "ctx"));
        // Hardware acquisition is already substituted. The real session owns this real child.
        Object session = field(suite, "childSession");
        invoke(session, "retain", new Class<?>[]{TeleOpTester.class}, fixture.owner);
        setField(suite, "inMenu", false);
        setField(suite, "activeTesterName", "Pod offsets");

        assertTrue(suite.onBackPressed());

        assertEquals("IDLE", fixture.phase());
        assertFalse((Boolean) field(fixture.owner, "started"));
        fixture.assertStopped();
        assertEquals(1, camera.closes);
        assertTrue((Boolean) field(suite, "inMenu"));
        assertTrue(suite.onBackPressed());
        suite.stop();
        assertEquals(1, camera.closes);
        for (RecordingOutput output : fixture.outputs) assertEquals(1, output.stops);
    }

    @Test
    public void timeoutPublishesInhibitionBeforeReentrantZeroFailureAndLeavesCameraForStop()
            throws Exception {
        for (boolean throwError : new boolean[]{false, true}) {
            Fixture fixture = enteredPhase("ROTATING");
            CountingCamera camera = new CountingCamera();
            setField(fixture.owner, "visionLane", camera.owned);
            setField(fixture.owner, "lastRecommendedStrafePodOffsetForwardInches", 12.0);
            setField(fixture.owner, "lastDxStartBodyInches", 4.0);
            Throwable expected = throwError ? new AssertionError("zero write error")
                    : new IllegalStateException("zero write failed");
            fixture.outputs[0].beforeZero = () -> {
                try {
                    assertEquals("IDLE", fixture.phase());
                    assertFalse((Boolean) field(fixture.owner, "autoStartRequested"));
                    assertNull(field(fixture.owner,
                            "lastRecommendedStrafePodOffsetForwardInches"));
                    assertNull(field(fixture.owner, "lastDxStartBodyInches"));
                    assertEquals(fixture.clock.cycle(),
                            ((Long) field(fixture.owner, "motionInhibitedCycle")).longValue());
                    fixture.queuePrimary();
                    fixture.queueAuto();
                    setField(fixture.owner, "resetRequested", true);
                    int beforeReentry = fixture.commands.size();
                    fixture.owner.loop(0.0);
                    assertEquals("IDLE", fixture.phase());
                    assertEquals("reentry must not write even another zero", beforeReentry,
                            fixture.commands.size());
                    assertFalse((Boolean) field(fixture.owner, "primaryActionRequested"));
                    assertFalse((Boolean) field(fixture.owner, "autoStartRequested"));
                    assertFalse((Boolean) field(fixture.owner, "resetRequested"));
                } catch (Exception failure) {
                    throw new AssertionError(failure);
                }
                if (expected instanceof Error) throw (Error) expected;
                throw (RuntimeException) expected;
            };
            fixture.queueAuto();
            try {
                fixture.loopAt(LIMIT);
                fail("Expected the real drive owner to propagate the zero failure");
            } catch (RuntimeException | Error failure) {
                assertSame(expected, failure);
            }
            assertEquals("IDLE", fixture.phase());
            assertSame(camera.owned, field(fixture.owner, "visionLane"));
            assertEquals(0, camera.closes);

            fixture.owner.stop();
            assertEquals(1, camera.closes);
            assertNull(field(fixture.owner, "visionLane"));
            fixture.assertStopped();
            fixture.owner.stop();
            assertEquals(1, camera.closes);
        }
    }

    private static Fixture enteredPhase(String phase) throws Exception {
        Fixture fixture = new Fixture(!"ROTATING".equals(phase), true);
        if ("SEARCH_TAG_END".equals(phase)) {
            fixture.startAutoWithTag();
            fixture.device.pose = sdkPose(Math.PI);
            fixture.loopAt(1.0);
        } else {
            fixture.queueAuto();
            fixture.owner.loop(0.0);
        }
        assertEquals(phase, fixture.phase());
        return fixture;
    }

    /** Shared hardware-substitution fixture; never supplies an already-matched endpoint. */
    static final class Fixture {
        private static final double[] TAG_X = {24.0, -24.0, 0.0, 0.0, 100.0, 100.0, 124.0, 76.0};
        private static final double[] TAG_Y = {0.0, 0.0, 24.0, -24.0, 26.0, 74.0, 50.0, 50.0};
        final LoopClock clock = new LoopClock();
        final Gamepad gamepad = new Gamepad();
        final List<Double> commands = new ArrayList<>();
        final List<String> telemetry = new ArrayList<>();
        final CalibrationReportTest.Downloads downloads = new CalibrationReportTest.Downloads();
        final RecordingOutput[] outputs = new RecordingOutput[4];
        final FakePinpoint device = new FakePinpoint();
        final PinpointOdometryPredictor predictor;
        final PinpointPodOffsetCalibrator owner;
        final PinpointPodOffsetCalibrator.Config config;
        ScriptedCamera camera = new ScriptedCamera();

        Fixture(boolean assist, boolean powered) throws Exception {
            this(assist, powered, config -> { });
        }

        Fixture(boolean assist, boolean powered,
                Consumer<PinpointPodOffsetCalibrator.Config> configure) throws Exception {
            clock.reset(0.0);
            config = PinpointPodOffsetCalibrator.Config.defaults();
            config.mecanum = powered ? FtcDrives.MecanumConfig.defaults() : null;
            config.automaticPhaseTimeoutSec = LIMIT;
            configure.accept(config);
            SimpleTagLayout layout = new SimpleTagLayout();
            for (int i = 0; i < TAG_X.length; i++) {
                layout.addPose(i + 1, new Pose3d(TAG_X[i], TAG_Y[i], 0.0, 0.0, 0.0, 0.0));
            }
            config.fixedTagLayout = layout;
            owner = new PinpointPodOffsetCalibrator(config,
                    assist ? ignored -> hardwareMap -> null : null);
            Telemetry sink = (Telemetry) Proxy.newProxyInstance(
                    Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                    (ignored, method, args) -> {
                        if ("clearAll".equals(method.getName())) telemetry.clear();
                        if (args != null && ("addData".equals(method.getName())
                                || "addLine".equals(method.getName()))) {
                            for (Object arg : args) telemetry.add(String.valueOf(arg));
                        }
                        return defaultValue(method.getReturnType());
                    });
            TesterContext context = new TesterContext(null, sink, gamepad, new Gamepad(), clock, downloads);
            setField(owner, "ctx", context);
            setField(owner, "clock", clock);
            setField(owner, "gamepads", Gamepads.create(context.gamepad1, context.gamepad2));
            predictor = device.newPredictor(config.pinpoint);
            setField(owner, "pinpoint", predictor);
            if (powered) {
                for (int i = 0; i < outputs.length; i++) outputs[i] = new RecordingOutput(commands);
                setField(owner, "drive", new MecanumDrivebase(outputs[0], outputs[1],
                        outputs[2], outputs[3], MecanumDrivebase.Config.defaults()));
            }
            if (assist) {
                installCamera();
                setField(owner, "assistOdometryHistory", new PlanarPoseHistory(predictor,
                        config.assistOdometryHistory));
            }
            predictor.update(clock);
            setField(owner, "latestPinpointPose", predictor.getEstimate().toPose2d());
            owner.start();
        }

        void installCamera() throws Exception {
            setField(owner, "visionLane", camera.owned);
            setField(owner, "tagSensor", camera.sensor);
            setField(owner, "tagEstimator", new AprilTagPoseEstimator(camera.sensor,
                    config.fixedTagLayout,
                    config.aprilTags.toAprilTagPoseEstimatorConfig(camera.cameraMountConfig())));
        }

        void queueAuto() throws Exception { setField(owner, "autoStartRequested", true); }
        void queuePrimary() throws Exception { setField(owner, "primaryActionRequested", true); }

        void startAutoWithTag() throws Exception {
            frameAt(clock.nowTimestamp(), Pose2d.zero());
            queueAuto();
            owner.loop(0.0);
            assertEquals("ROTATING", phase());
            camera.next = AprilTagDetections.none();
        }

        void frameAt(LoopTimestamp timestamp, Pose2d fieldToRobot) {
            double c = Math.cos(fieldToRobot.headingRad);
            double s = Math.sin(fieldToRobot.headingRad);
            int selected = -1;
            double nearestRange = Double.POSITIVE_INFINITY;
            for (int i = 0; i < TAG_X.length; i++) {
                double dx = TAG_X[i] - fieldToRobot.xInches;
                double dy = TAG_Y[i] - fieldToRobot.yInches;
                double forward = c * dx + s * dy - 1.0;
                double left = -s * dx + c * dy;
                double range = Math.hypot(forward, left);
                if (forward > Math.abs(left) && range < nearestRange) {
                    selected = i;
                    nearestRange = range;
                }
            }
            assertTrue("fixture needs a fixed tag visibly in front of this camera", selected >= 0);
            double dx = TAG_X[selected] - fieldToRobot.xInches;
            double dy = TAG_Y[selected] - fieldToRobot.yInches;
            // Independently invert the planar robot pose and subtract the camera's +1 in mount.
            Pose3d cameraToTag = new Pose3d(c * dx + s * dy - 1.0,
                    -s * dx + c * dy, 0.0, -fieldToRobot.headingRad, 0.0, 0.0);
            camera.next = AprilTagDetections.fromFrame(timestamp,
                    Collections.singletonList(AprilTagObservation.target(selected + 1, cameraToTag)));
        }

        void noFrame() { camera.next = AprilTagDetections.none(); }

        void poseAt(double timeSec, Pose2d pose) {
            device.pose = sdkPose(pose);
            loopAt(timeSec);
        }

        void loopAt(double timeSec) {
            clock.update(timeSec);
            owner.loop(clock.dtSec());
        }

        String phase() throws Exception { return String.valueOf(field(owner, "phase")); }

        void assertPowered() {
            boolean nonzero = false;
            for (RecordingOutput output : outputs) nonzero |= output.power != 0.0;
            assertTrue("expected a recorded nonzero drive command", nonzero);
        }

        void assertStopped() {
            for (RecordingOutput output : outputs) {
                if (output != null) assertEquals(0.0, output.power, 0.0);
            }
        }

        void assertOnlyZeroCommandsSince(int index) {
            for (int i = index; i < commands.size(); i++) assertEquals(0.0, commands.get(i), 0.0);
            assertStopped();
        }

        void assertRejected() throws Exception {
            assertEquals("IDLE", phase());
            assertNull(field(owner, "lastRecommendedStrafePodOffsetForwardInches"));
            assertNull(field(owner, "lastRecommendedForwardPodOffsetLeftInches"));
            assertStopped();
            String reason = (String) field(owner, "lastAttemptFailure");
            assertNotNull("rejection must retain its own failure, not rely on unrelated rows", reason);
            assertTrue(reason, reason.contains("Attempt discarded"));
            assertTrue(reason, reason.contains("timed out") || reason.contains("invalid elapsed time"));
            assertTrue(reason, reason.startsWith("ROTATING ")
                    || reason.startsWith("SEARCH_TAG_START ")
                    || reason.startsWith("SEARCH_TAG_END "));
            assertTrue("the exact retained reason must render without completed results: " + telemetry,
                    telemetry.contains(reason));
        }
    }

    static final class FakePinpoint {
        Pose2D pose = sdkPose(0.0);
        GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.READY;
        int polls;
        int rebases;
        double configuredForwardPodOffsetLeftInches;
        double configuredStrafePodOffsetForwardInches;

        PinpointOdometryPredictor newPredictor(PinpointOdometryPredictor.Config config)
                throws Exception {
            Class<?> deviceType = Class.forName(PinpointOdometryPredictor.class.getName()
                    + "$PinpointDevice");
            Class<?> lookupType = Class.forName(PinpointOdometryPredictor.class.getName()
                    + "$PinpointDeviceLookup");
            Object fake = Proxy.newProxyInstance(deviceType.getClassLoader(),
                    new Class<?>[]{deviceType}, (ignored, method, args) -> {
                        switch (method.getName()) {
                            case "setOffsetsInches":
                                configuredForwardPodOffsetLeftInches = (Double) args[0];
                                configuredStrafePodOffsetForwardInches = (Double) args[1];
                                return null;
                            case "update": polls++; return null;
                            case "getDeviceStatus": return status;
                            case "getPosition": return pose;
                            case "setPosition": rebases++; pose = (Pose2D) args[0]; return null;
                            default: return defaultValue(method.getReturnType());
                        }
                    });
            Object lookup = Proxy.newProxyInstance(lookupType.getClassLoader(),
                    new Class<?>[]{lookupType}, (ignored, method, args) -> fake);
            Constructor<PinpointOdometryPredictor> constructor =
                    PinpointOdometryPredictor.class.getDeclaredConstructor(
                            lookupType, PinpointOdometryPredictor.Config.class);
            constructor.setAccessible(true);
            return constructor.newInstance(lookup, config);
        }
    }

    /** The selected camera resource is real-owned; only its external frame/readiness is scripted. */
    static final class ScriptedCamera implements AprilTagVision, AutoCloseable {
        final OwnedAprilTagCamera owned = new OwnedAprilTagCamera(this, this);
        AprilTagDetections next = AprilTagDetections.none();
        int polls;
        int closes;
        RuntimeException failure;
        final AprilTagSensor sensor = ignored -> { polls++; return next; };

        @Override public AprilTagSensor tagSensor() { return sensor; }
        @Override public CameraMountConfig cameraMountConfig() {
            return CameraMountConfig.of(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        @Override public VisionReadiness readiness(LoopClock clock) {
            if (failure != null) throw failure;
            return VisionReadiness.ready();
        }
        @Override public void close() { closes++; }
    }

    private static final class RecordingOutput implements PowerOutput {
        final List<Double> commands;
        double power;
        int stops;
        Runnable beforeZero;

        RecordingOutput(List<Double> commands) { this.commands = commands; }

        @Override public void setPower(double value) {
            commands.add(value);
            if (value == 0.0 && beforeZero != null) beforeZero.run();
            power = value;
        }

        @Override public double getCommandedPower() { return power; }
        @Override public void stop() { stops++; power = 0.0; }
    }

    private static final class CountingCamera implements AprilTagVision, AutoCloseable {
        final OwnedAprilTagCamera owned = new OwnedAprilTagCamera(this, this);
        int closes;

        @Override public AprilTagSensor tagSensor() { return ignored -> AprilTagDetections.none(); }
        @Override public CameraMountConfig cameraMountConfig() { return CameraMountConfig.identity(); }
        @Override public VisionReadiness readiness(LoopClock clock) {
            return VisionReadiness.notReady("scripted camera has no observations");
        }
        @Override public void close() { closes++; }
    }

    private static Pose2D sdkPose(double headingRad) {
        return new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, headingRad);
    }

    static Pose2D sdkPose(Pose2d pose) {
        return new Pose2D(DistanceUnit.INCH, pose.xInches, pose.yInches,
                AngleUnit.RADIANS, pose.headingRad);
    }

    private static Field reflectedField(Object owner, String name) throws Exception {
        for (Class<?> type = owner.getClass(); type != null; type = type.getSuperclass()) {
            try {
                Field field = type.getDeclaredField(name);
                field.setAccessible(true);
                return field;
            } catch (NoSuchFieldException ignored) {
                // Runtime context belongs to BaseTeleOpTester; calibration state belongs locally.
            }
        }
        throw new NoSuchFieldException(name);
    }

    static Object field(Object owner, String name) throws Exception {
        return reflectedField(owner, name).get(owner);
    }

    static void setField(Object owner, String name, Object value) throws Exception {
        reflectedField(owner, name).set(owner, value);
    }

    static Object invoke(Object owner, String name) throws Exception {
        return invoke(owner, name, new Class<?>[0]);
    }

    private static Object invoke(Object owner, String name, Class<?>[] types, Object... args)
            throws Exception {
        Method method = owner.getClass().getDeclaredMethod(name, types);
        method.setAccessible(true);
        try {
            return method.invoke(owner, args);
        } catch (InvocationTargetException failure) {
            Throwable cause = failure.getCause();
            if (cause instanceof RuntimeException) throw (RuntimeException) cause;
            if (cause instanceof Error) throw (Error) cause;
            throw failure;
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (type == boolean.class) return false;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0.0f;
        if (type == double.class) return 0.0;
        if (type == char.class) return '\0';
        return null;
    }
}
