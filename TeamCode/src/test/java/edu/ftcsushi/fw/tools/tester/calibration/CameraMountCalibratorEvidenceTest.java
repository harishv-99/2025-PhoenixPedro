package edu.ftcsushi.fw.tools.tester.calibration;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.ftc.vision.AprilTagVision;
import edu.ftcsushi.fw.ftc.vision.AprilTagCameraFactory;
import edu.ftcsushi.fw.ftc.vision.OwnedAprilTagCamera;
import edu.ftcsushi.fw.ftc.vision.VisionReadiness;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Real tester initialization, bindings, gamepad edges, clock and camera ownership over a scripted
 * camera boundary. No motor, camera hardware, optical timing or physical accuracy is established.
 * Private inspection observes the retained result; it does not inject a capture or solved pose.
 */
public final class CameraMountCalibratorEvidenceTest {
    private static final double EPS = 1e-9;

    @Test
    public void captureUsesThisFrameInsteadOfThePreviousPreview() {
        Fixture f = new Fixture();
        f.observeAt(0.02, observation(24.0));
        f.gamepad.a = true;
        f.observeAt(0.03, observation(20.0));

        assertEquals(1, f.sampleCount());
        // Tag at field X=30, known robot X=0: current camera-to-tag X=20 means mount X=10.
        assertEquals(10.0, f.mean().xInches, EPS);
        f.owner.stop();
    }

    @Test
    public void staleFrameCannotCaptureThePreviouslyFreshPreview() {
        Fixture f = new Fixture();
        f.observeAt(0.02, observation(24.0));
        f.gamepad.a = true;
        f.loopAt(0.50);

        assertEquals(0, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void distinctButtonEdgesCannotCountTheSameCaptureTwice() {
        Fixture f = new Fixture();
        f.observeAt(0.02, observation(24.0));
        f.pressA();
        assertEquals(1, f.sampleCount());
        f.releaseA();
        f.pressA();

        assertEquals(1, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void clearWinsOverCaptureInTheSameBindingTraversal() {
        Fixture f = new Fixture();
        f.observeAt(0.02, observation(24.0));
        f.pressA();
        f.releaseA();
        f.gamepad.a = true;
        f.gamepad.b = true;
        f.loopAt(0.05);

        assertEquals(0, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void driverStationStartDiscardsThePriorClockEpochBatchAndPreview() {
        Fixture f = new Fixture();
        f.observeAt(0.02, observation(24.0));
        f.pressA();
        f.releaseA();
        f.startRun();
        f.gamepad.a = true;
        f.loopAt(0.01);

        assertEquals(0, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void failedCameraReplacementCannotInheritTheOldOwnersAverage() {
        Fixture f = new Fixture();
        f.observeAt(0.02, observation(24.0));
        f.pressA();
        f.releaseA();
        ScriptedCamera previous = f.camera;
        previous.readinessFailure = new IllegalStateException("lost camera");
        f.loopAt(0.05);
        assertEquals(1, previous.closes);
        ScriptedCamera replacement = new ScriptedCamera();
        f.nextCamera = replacement;
        f.loopAt(0.06);
        f.gamepad.a = true;
        f.loopAt(0.07); // The actual configured-device picker opens the next owned lane.

        assertEquals(2, f.opens);
        assertSame(replacement, f.camera);
        assertEquals(0, f.sampleCount());
        f.owner.stop();
        assertEquals(1, replacement.closes);
    }

    @Test
    public void equalCaptureTimesWithDifferentObjectsAndOlderFramesCannotGrowTheBatch() {
        Fixture f = new Fixture();
        f.observeAt(0.10, observation(24.0));
        LoopTimestamp accepted = f.camera.next.frameTimestamp();
        LoopTimestamp equalTime = f.clock.nowTimestamp();
        f.pressA();
        f.releaseA();
        assertNotSame(accepted, equalTime);
        assertEquals(0.0, equalTime.secondsSince(accepted), 0.0);
        f.frameAt(equalTime, 1, observation(20.0));
        f.pressA();
        assertEquals(1, f.sampleCount());
        assertEquals(6.0, f.mean().xInches, EPS);
        f.releaseA();
        f.frameAt(f.clock.timestampSecondsAgo(f.clock.nowSec() - 0.09), 1, observation(18.0));
        f.pressA();
        assertEquals(1, f.sampleCount());
        f.releaseA();
        f.gamepad.a = true;
        f.observeAt(0.20, observation(20.0));
        assertEquals(2, f.sampleCount());
        assertEquals(8.0, f.mean().xInches, EPS);
        f.owner.stop();
    }

    @Test
    public void inclusiveAgeBoundaryAcceptsButOlderEmptyAndUnavailableFramesDoNotWait() {
        for (int condition = 0; condition < 4; condition++) {
            CameraMountCalibrator.Config config = Fixture.config(defaultLayout());
            config.maxDetectionAgeSec = 0.25;
            Fixture f = new Fixture(config);
            f.observeAt(0.125, observation(24.0));
            if (condition == 2) f.camera.next = AprilTagDetections.fromFrame(
                    f.clock.nowTimestamp(), Collections.emptyList());
            if (condition == 3) f.camera.next = AprilTagDetections.none();
            f.gamepad.a = true;
            f.loopAt(condition == 1 ? 0.375001 : 0.375);
            assertEquals("age/availability condition " + condition,
                    condition == 0 ? 1 : 0, f.sampleCount());
            if (condition != 0) {
                assertNull(field(f.owner, "lastRobotToCameraSample"));
                f.observeAt(0.40, observation(20.0)); // Still-held A is not a deferred request.
                assertEquals(0, f.sampleCount());
                f.releaseA();
                f.gamepad.a = true;
                f.observeAt(0.45, observation(20.0));
                assertEquals(1, f.sampleCount());
            }
            f.owner.stop();
        }
    }

    @Test
    public void clearRejectsOldAndSameBoundaryFramesAndRequiresANewButtonEdge() {
        Fixture f = capturedFixture();
        f.gamepad.a = true;
        f.gamepad.b = true;
        f.observeAt(0.05, observation(20.0));
        LoopTimestamp clearBoundary = f.clock.nowTimestamp();
        assertEquals(0, f.sampleCount());
        f.loopCurrent(); // Neither another owner call nor later A registration can undo B.
        assertEquals(0, f.sampleCount());
        f.gamepad.b = false;
        f.observeAt(0.06, observation(20.0));
        assertEquals(0, f.sampleCount());
        f.releaseA();
        f.frameAt(clearBoundary, 1, observation(20.0));
        f.pressA();
        assertEquals("capture at the clear boundary is not a later image", 0, f.sampleCount());
        f.releaseA();
        f.gamepad.a = true;
        f.observeAt(0.10, observation(20.0));
        assertEquals(1, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void tagAndKnownPoseEditsClearTheBatchAndCannotReinterpretAnOldImage() {
        List<Consumer<Gamepad>> edits = new ArrayList<>();
        edits.add(g -> g.y = true);
        edits.add(g -> g.dpad_right = true);
        edits.add(g -> g.dpad_up = true);
        edits.add(g -> g.left_bumper = true);
        for (Consumer<Gamepad> edit : edits) {
            Fixture f = capturedFixture();
            LoopTimestamp beforeEdit = f.camera.next.frameTimestamp();
            edit.accept(f.gamepad);
            f.gamepad.a = true;
            f.observeAt(0.05, observation(20.0));
            assertEquals(0, f.sampleCount());
            assertNull(field(f.owner, "lastRobotToCameraSample"));
            neutral(f.gamepad);
            f.loopAt(0.06);
            int selected = (Integer) field(f.owner, "selectedTagId");
            f.frameAt(beforeEdit, selected, observation(20.0));
            f.pressA();
            assertEquals("an old image cannot be solved under newly edited setup", 0, f.sampleCount());
            f.releaseA();
            f.clock.update(0.10);
            f.frameNow(selected, observation(20.0));
            f.gamepad.a = true;
            f.loopCurrent();
            assertEquals(1, f.sampleCount());
            f.owner.stop();
        }
    }

    @Test
    public void stepModeFieldNavigationAndANoopTagEditPreserveTheAcceptedBatch() {
        Fixture f = capturedFixture();
        f.gamepad.x = true; // Tag 1 decremented with a lower bound of 1 is not a setup change.
        f.loopAt(0.05);
        assertEquals(1, f.sampleCount());
        neutral(f.gamepad);
        f.loopAt(0.06);
        f.gamepad.start = true; // Gamepad START, not Driver Station START.
        f.loopAt(0.07);
        assertEquals(1, f.sampleCount());
        neutral(f.gamepad);
        f.loopAt(0.08);
        f.gamepad.right_stick_button = true;
        f.loopAt(0.09);
        assertEquals(1, f.sampleCount());
        neutral(f.gamepad);
        f.loopAt(0.10);
        f.gamepad.dpad_up = true; // In edit mode this selects a field, not a new value.
        f.loopAt(0.11);
        assertEquals(1, f.sampleCount());
        neutral(f.gamepad);
        f.loopAt(0.12);
        f.gamepad.dpad_right = true; // The selected TAG_ID field now changes its actual value.
        f.loopAt(0.13);
        assertEquals(0, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void temporaryWaitingRetainsOnlyHistoricalSamplesAndDropsCaptureIntent() {
        Fixture f = capturedFixture();
        Pose3d historical = f.mean();
        f.camera.readiness = VisionReadiness.notReady("temporarily waiting");
        f.gamepad.a = true;
        f.observeAt(0.05, observation(20.0));
        assertEquals(1, f.sampleCount());
        assertEquals(historical.xInches, f.mean().xInches, EPS);
        assertNull(field(f.owner, "lastRobotToCameraSample"));
        assertTrue(f.telemetry.contains("Historical captured samples (not current readiness)"));
        f.camera.readiness = VisionReadiness.ready();
        f.observeAt(0.06, observation(20.0));
        assertEquals(1, f.sampleCount());
        f.releaseA();
        f.gamepad.a = true;
        f.observeAt(0.10, observation(20.0));
        assertEquals(2, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void aClockResetWithoutStartAlsoDiscardsOldBatchAndDelayedFrames() {
        Fixture f = capturedFixture();
        f.clock.reset(2.0);
        f.gamepad.a = true;
        f.loopCurrent();
        assertEquals(0, f.sampleCount());
        assertNull(field(f.owner, "lastRobotToCameraSample"));
        f.releaseA();
        f.gamepad.a = true;
        f.observeAt(2.05, observation(20.0));
        assertEquals(1, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void framesAtOrBeforeCameraOpeningCannotBecomeSamples() {
        Fixture f = new Fixture();
        f.frameAt(f.clock.timestampSecondsAgo(f.clock.nowSec()), 1, observation(24.0));
        f.pressA();
        assertEquals(0, f.sampleCount());
        f.releaseA();
        f.gamepad.a = true;
        f.observeAt(0.05, observation(24.0));
        assertEquals(1, f.sampleCount());
        f.owner.stop();
    }

    @Test
    public void nonfiniteObservationGeometryNeverReplacesAValidBatchOrPoisonsRecovery() {
        Pose3d[] invalid = {
                new Pose3d(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0),
                new Pose3d(24.0, Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0, 0.0),
                new Pose3d(24.0, 0.0, Double.NEGATIVE_INFINITY, 0.0, 0.0, 0.0),
                new Pose3d(24.0, 0.0, 0.0, Double.NaN, 0.0, 0.0),
                new Pose3d(24.0, 0.0, 0.0, 0.0, Double.NaN, 0.0),
                new Pose3d(24.0, 0.0, 0.0, 0.0, 0.0, Double.POSITIVE_INFINITY)
        };
        for (Pose3d bad : invalid) {
            Fixture f = capturedFixture();
            f.gamepad.a = true;
            f.observeAt(0.05, bad);
            assertEquals(1, f.sampleCount());
            assertEquals(6.0, f.mean().xInches, EPS);
            assertNull(field(f.owner, "lastRobotToCameraSample"));
            f.releaseA();
            f.gamepad.a = true;
            f.observeAt(0.08, observation(20.0));
            assertEquals(2, f.sampleCount());
            assertEquals(8.0, f.mean().xInches, EPS);
            f.owner.stop();
        }
    }

    @Test
    public void nullWrongClockAndThrowingSourcesFailWithoutLeavingACaptureQueued() {
        for (int condition = 0; condition < 3; condition++) {
            Fixture f = new Fixture();
            f.observeAt(0.02, observation(24.0));
            RuntimeException expected = new IllegalStateException("camera read failed");
            if (condition == 0) f.camera.next = null;
            if (condition == 1) {
                LoopClock other = new LoopClock();
                other.reset(0.0);
                f.frameAt(other.nowTimestamp(), 1, observation(20.0));
            }
            if (condition == 2) f.camera.readFailure = expected;
            f.gamepad.a = true;
            try {
                f.loopAt(0.03);
                fail("Expected source contract failure " + condition);
            } catch (RuntimeException failure) {
                if (condition == 2) assertSame(expected, failure);
                else assertNotNull(failure.getMessage());
            }
            assertEquals(0, f.sampleCount());
            assertNull(field(f.owner, "lastRobotToCameraSample"));
            f.camera.readFailure = null;
            // This harness deliberately permits a later cycle after failure; a host may stop instead.
            f.observeAt(0.04, observation(20.0));
            assertEquals(0, f.sampleCount());
            f.releaseA();
            f.gamepad.a = true;
            f.observeAt(0.07, observation(20.0));
            assertEquals(1, f.sampleCount());
            f.owner.stop();
        }
    }

    @Test
    public void reentrantClearStopAndClockResetDuringReadinessOrReadCannotPublishTheOldAttempt() {
        for (boolean readinessCallback : new boolean[]{false, true}) {
            for (int callbackAction = 0; callbackAction < 3; callbackAction++) {
                final int action = callbackAction;
                Fixture f = capturedFixture();
                ScriptedCamera camera = f.camera;
                Runnable callback = () -> {
                    if (action == 0) invoke(f.owner, "clearCapturedSamples");
                    if (action == 1) f.owner.stop();
                    if (action == 2) f.clock.reset(2.0);
                    // Reset is detected when the external callback returns to its owner.
                    if (action != 2) {
                        assertEquals(0, f.sampleCount());
                        assertNull(field(f.owner, "lastRobotToCameraSample"));
                    }
                };
                // Clear invokes the private action registered by B, not fabricated binding reentry.
                if (readinessCallback) camera.duringReadiness = callback;
                else camera.duringRead = callback;
                f.gamepad.a = true;
                f.observeAt(0.05, observation(20.0));
                assertEquals(0, f.sampleCount());
                assertNull(field(f.owner, "lastRobotToCameraSample"));
                if (action == 1) {
                    assertEquals(1, camera.closes);
                    int reads = camera.reads;
                    f.loopAt(0.06);
                    assertEquals(reads, camera.reads);
                } else {
                    f.releaseA();
                    f.gamepad.a = true;
                    f.observeAt(action == 2 ? 2.05 : 0.08, observation(20.0));
                    assertEquals(1, f.sampleCount());
                }
                f.owner.stop();
                assertEquals(1, camera.closes);
            }
        }
    }

    @Test
    public void backStopAndReadinessFailureClearEvidenceBeforeCameraCloseCallbacks() {
        for (int ending = 0; ending < 3; ending++) {
            Fixture f = capturedFixture();
            ScriptedCamera camera = f.camera;
            camera.duringClose = () -> {
                assertEquals(0, f.sampleCount());
                assertNull(f.mean());
                assertNull(field(f.owner, "lastRobotToCameraSample"));
                assertNull(field(f.owner, "lastObservedCameraToTag"));
                f.owner.stop(); // Existing detach-once lifecycle must still tolerate reentry.
            };
            if (ending == 0) assertTrue(f.owner.onBackPressed());
            if (ending == 1) f.owner.stop();
            if (ending == 2) {
                camera.readinessFailure = new IllegalStateException("readiness lost");
                f.loopAt(0.05);
            }
            assertEquals(0, f.sampleCount());
            assertEquals(1, camera.closes);
            f.owner.stop();
            assertEquals(1, camera.closes);
        }
    }

    @Test
    public void openingCallbacksMayClearOrResetEvidenceWithoutStrandingCameraSetup() {
        for (int callbackStage = 0; callbackStage < 3; callbackStage++) {
            for (boolean resetClock : new boolean[]{false, true}) {
                final int stage = callbackStage;
                Fixture f = new Fixture(Fixture.config(defaultLayout()), fixture -> {
                    Runnable callback = () -> {
                        if (resetClock) fixture.clock.reset(0.0);
                        else invoke(fixture.owner, "clearCapturedSamples");
                    };
                    if (stage == 0) fixture.duringOpen = callback;
                    if (stage == 1) fixture.camera.duringSensorAccess = callback;
                    if (stage == 2) fixture.duringDescription = callback;
                });
                assertEquals(1, f.opens);
                assertEquals("clear/reset changes sample evidence, not camera ownership", 0,
                        f.camera.closes);
                f.observeAt(0.02, observation(24.0));
                f.pressA();
                assertEquals("setup stage=" + stage + ", clock reset=" + resetClock,
                        1, f.sampleCount());
                assertEquals(6.0, f.mean().xInches, EPS);
                f.owner.stop();
                assertEquals(1, f.camera.closes);
            }
        }
    }

    @Test
    public void stopDuringOpenOrAccessorsCannotResurrectTheReturnedCamera() {
        for (int callbackStage = 0; callbackStage < 3; callbackStage++) {
            final int stage = callbackStage;
            Fixture f = new Fixture(Fixture.config(defaultLayout()), fixture -> {
                Runnable callback = fixture.owner::stop;
                if (stage == 0) fixture.duringOpen = callback;
                if (stage == 1) fixture.camera.duringSensorAccess = callback;
                if (stage == 2) fixture.duringDescription = callback;
            });
            assertEquals(1, f.opens);
            assertEquals(1, f.camera.closes);
            assertNull(field(f.owner, "visionLane"));
            assertNull(field(f.owner, "tagSensor"));
            assertEquals(0, f.sampleCount());
            int reads = f.camera.reads;
            f.gamepad.a = true;
            f.observeAt(0.02, observation(20.0));
            assertEquals(reads, f.camera.reads);
            assertEquals(0, f.sampleCount());
            f.owner.stop();
            assertEquals(1, f.camera.closes);
        }
    }

    @Test
    public void lateOpenAfterReentrantStopRetainsAnUncertainCloseAndNeverReopens() {
        RuntimeException failedClose = new IllegalStateException("late owner close failed");
        Fixture f = new Fixture(Fixture.config(defaultLayout()), fixture -> {
            fixture.duringOpen = fixture.owner::stop;
            fixture.camera.closeFailure = failedClose;
        });
        assertEquals(1, f.camera.closes);
        assertNull(field(f.owner, "visionLane"));
        assertSame(failedClose, field(f.owner, "visionFailure"));
        assertTrue((Boolean) field(f.owner, "visionCleanupFailed"));
        assertEquals(0, f.sampleCount());
        f.nextCamera = new ScriptedCamera();
        f.gamepad.a = true;
        f.loopAt(0.02);
        assertEquals(1, f.opens);
        f.owner.stop();
        assertEquals("a detached uncertain owner is never closed a second time", 1, f.camera.closes);
    }

    @Test
    public void ambiguousCapturedRotationsCountDistinctFramesButDoNotPrintARecommendation() {
        // Deliberately synthetic finite transforms: this is not a claim of optical visibility.
        Fixture f = new Fixture(new SimpleTagLayout().addPose(1, Pose3d.zero()));
        f.observeAt(0.02, Pose3d.zero());
        f.pressA();
        assertEquals(1, f.sampleCount());
        assertTrue(f.telemetry.stream().anyMatch(line -> line.startsWith("CameraMountConfig.of(")));
        f.releaseA();
        f.gamepad.a = true;
        f.observeAt(0.05, new Pose3d(0.0, 0.0, 0.0, Math.PI, 0.0, 0.0));
        assertEquals(2, f.sampleCount());
        assertNull(f.mean());
        assertTrue(f.telemetry.stream().anyMatch(line -> line.startsWith("Average unavailable:")));
        assertTrue(f.telemetry.stream().anyMatch(line -> line.contains("ambiguous")));
        assertFalse(f.telemetry.stream().anyMatch(line -> line.startsWith("CameraMountConfig.")));
        Object accepted = field(f.owner, "lastAcceptedFrame");
        f.releaseA();
        f.pressA(); // Another press on the same accepted ambiguous image is still a duplicate.
        assertEquals(2, f.sampleCount());
        assertSame(accepted, field(f.owner, "lastAcceptedFrame"));
        assertNull(f.mean());
        assertFalse(f.telemetry.stream().anyMatch(line -> line.startsWith("CameraMountConfig.")));
        f.releaseA();
        f.gamepad.a = true;
        f.observeAt(0.10, Pose3d.zero());
        assertEquals(3, f.sampleCount());
        assertNotNull(f.mean());
        assertEquals(1.0, Math.cos(f.mean().yawRad), EPS);
        assertEquals(0.0, Math.sin(f.mean().yawRad), EPS);
        assertEquals(0.0, f.mean().pitchRad, EPS);
        assertEquals(0.0, f.mean().rollRad, EPS);
        assertTrue(f.telemetry.stream().anyMatch(line -> line.startsWith("CameraMountConfig.of(")));
        f.owner.stop();
    }

    private static Fixture capturedFixture() {
        Fixture f = new Fixture(defaultLayout());
        f.observeAt(0.02, observation(24.0));
        f.pressA();
        assertEquals(1, f.sampleCount());
        f.releaseA();
        return f;
    }

    private static TagLayout defaultLayout() {
        return new SimpleTagLayout()
                .addPose(1, new Pose3d(30.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .addPose(2, new Pose3d(40.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    private static void neutral(Gamepad gamepad) {
        gamepad.a = gamepad.b = gamepad.x = gamepad.y = false;
        gamepad.dpad_up = gamepad.dpad_down = gamepad.dpad_left = gamepad.dpad_right = false;
        gamepad.left_bumper = gamepad.right_bumper = false;
        gamepad.start = gamepad.right_stick_button = false;
    }

    private static Pose3d observation(double x) {
        return new Pose3d(x, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /** Shared only with the independent geometry suite; all production construction remains real. */
    static final class Fixture {
        final LoopClock clock = new LoopClock();
        final Gamepad gamepad = new Gamepad();
        final List<String> telemetry = new ArrayList<>();
        final CameraMountCalibrator owner;
        ScriptedCamera camera = new ScriptedCamera();
        ScriptedCamera nextCamera;
        Runnable duringOpen;
        Runnable duringDescription;
        int opens;
        boolean running;

        Fixture() {
            this(new SimpleTagLayout().addPose(1,
                    new Pose3d(30.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
        }

        Fixture(TagLayout layout) {
            this(config(layout));
        }

        Fixture(CameraMountCalibrator.Config config) {
            this(config, fixture -> { });
        }

        Fixture(CameraMountCalibrator.Config config, Consumer<Fixture> beforeInit) {
            clock.reset(0.0);
            owner = new CameraMountCalibrator(config, name -> new AprilTagCameraFactory() {
                @Override public OwnedAprilTagCamera open(HardwareMap hardwareMap) {
                    opens++;
                    if (nextCamera != null) {
                        camera = nextCamera;
                        nextCamera = null;
                    }
                    Runnable callback = duringOpen;
                    duringOpen = null;
                    if (callback != null) callback.run();
                    return camera.owned;
                }

                @Override public String description() {
                    Runnable callback = duringDescription;
                    duringDescription = null;
                    if (callback != null) callback.run();
                    return "Scripted camera";
                }
            });
            HardwareMap hardware = new HardwareMap(null, null);
            hardware.put("camera", new NamedDevice());
            Telemetry sink = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class}, (proxy, method, args) -> {
                        if ("clearAll".equals(method.getName())) telemetry.clear();
                        if (args != null && ("addData".equals(method.getName())
                                || "addLine".equals(method.getName()))) {
                            for (Object arg : args) telemetry.add(String.valueOf(arg));
                        }
                        return defaultValue(method.getReturnType());
                    });
            beforeInit.accept(this);
            owner.init(new TesterContext(hardware, sink, gamepad, new Gamepad(), clock));
            loopAt(0.01); // A neutral cycle arms the real REARM_AFTER_NEUTRAL control context.
        }

        static CameraMountCalibrator.Config config(TagLayout layout) {
            CameraMountCalibrator.Config config = CameraMountCalibrator.Config.defaults();
            config.preferredVisionDeviceName = "camera";
            config.visionDeviceType = HardwareDevice.class;
            config.visionPickerTitle = "Scripted camera";
            config.fixedTagLayout = layout;
            return config;
        }

        void frameAt(LoopTimestamp timestamp, int id, Pose3d cameraToTag) {
            camera.next = AprilTagDetections.fromFrame(timestamp,
                    Collections.singletonList(AprilTagObservation.target(id, cameraToTag)));
        }

        void frameNow(int id, Pose3d cameraToTag) {
            frameAt(clock.nowTimestamp(), id, cameraToTag);
        }

        void observeAt(double timeSec, Pose3d cameraToTag) {
            clock.update(timeSec);
            frameNow(1, cameraToTag);
            loopCurrent();
        }

        void loopAt(double timeSec) {
            clock.update(timeSec);
            loopCurrent();
        }

        void loopCurrent() {
            if (running) owner.loop(clock.dtSec());
            else owner.initLoop(clock.dtSec());
        }

        void pressA() {
            gamepad.a = true;
            loopAt(clock.nowSec() + 0.01);
        }

        void releaseA() {
            gamepad.a = false;
            loopAt(clock.nowSec() + 0.01);
        }

        void startRun() {
            clock.reset(0.0); // The FTC host owns this operation, never the calibrator.
            running = true;
            owner.start();
        }

        int sampleCount() {
            return (Integer) invoke(field(owner, "avg"), "count");
        }

        Pose3d mean() {
            return (Pose3d) invoke(field(owner, "avg"), "meanOrNull");
        }
    }

    /** One owned camera; only external observations, readiness and cleanup effects are scripted. */
    static final class ScriptedCamera implements AprilTagVision, AutoCloseable {
        final OwnedAprilTagCamera owned = new OwnedAprilTagCamera(this, this);
        AprilTagDetections next = AprilTagDetections.none();
        VisionReadiness readiness = VisionReadiness.ready();
        RuntimeException readinessFailure;
        RuntimeException readFailure;
        RuntimeException closeFailure;
        Runnable duringRead;
        Runnable duringSensorAccess;
        Runnable duringReadiness;
        Runnable duringClose;
        int reads;
        int closes;

        @Override public AprilTagSensor tagSensor() {
            Runnable accessorCallback = duringSensorAccess;
            duringSensorAccess = null;
            if (accessorCallback != null) accessorCallback.run();
            return clock -> {
                reads++;
                Runnable callback = duringRead;
                duringRead = null;
                if (callback != null) callback.run();
                if (readFailure != null) throw readFailure;
                return next;
            };
        }

        @Override public CameraMountConfig cameraMountConfig() {
            // The unknown mount must not be consulted by this calibration owner.
            throw new AssertionError("mount calibration must not read its configured answer");
        }

        @Override public VisionReadiness readiness(LoopClock clock) {
            Runnable callback = duringReadiness;
            duringReadiness = null;
            if (callback != null) callback.run();
            if (readinessFailure != null) throw readinessFailure;
            return readiness;
        }

        @Override public void close() {
            closes++;
            Runnable callback = duringClose;
            duringClose = null;
            if (callback != null) callback.run();
            if (closeFailure != null) throw closeFailure;
        }
    }

    private static final class NamedDevice implements HardwareDevice {
        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return "Scripted camera name"; }
        @Override public String getConnectionInfo() { return "test boundary"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }

    static Object field(Object target, String name) {
        try {
            Field field = target.getClass().getDeclaredField(name);
            field.setAccessible(true);
            return field.get(target);
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError(failure);
        }
    }

    static void setField(Object target, String name, Object value) {
        try {
            Field field = target.getClass().getDeclaredField(name);
            field.setAccessible(true);
            field.set(target, value);
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError(failure);
        }
    }

    static Object invoke(Object target, String name, Object... args) {
        for (Method method : target.getClass().getDeclaredMethods()) {
            if (!method.getName().equals(name) || method.getParameterTypes().length != args.length) {
                continue;
            }
            try {
                method.setAccessible(true);
                return method.invoke(target, args);
            } catch (InvocationTargetException failure) {
                Throwable cause = failure.getCause();
                if (cause instanceof RuntimeException) throw (RuntimeException) cause;
                if (cause instanceof Error) throw (Error) cause;
                throw new AssertionError(cause);
            } catch (ReflectiveOperationException failure) {
                throw new AssertionError(failure);
            }
        }
        throw new AssertionError("No method " + name);
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
}
