package edu.ftcsushi.fw.tools.tester;

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

import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.MecanumDrivebase;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.ftc.ui.HardwareNamePicker;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcsushi.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcsushi.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcsushi.fw.ftc.vision.VisionReadiness;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcsushi.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcsushi.fw.tools.tester.localization.PinpointAprilTagCorrectedLocalizationTester;

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

            retryFreshSelection(kind, owner);

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
            if (kind == OwnerKind.PINPOINT_CORRECTED) {
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
            if (kind == OwnerKind.PINPOINT_CORRECTED) {
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
    public void cleanFactoryOpenFailureHasNoOwnerToCloseAndAllowsFreshSelection() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException failure = new IllegalStateException(kind + " open failed");
            OpenProbe opener = OpenProbe.failingOpen(failure);
            TeleOpTester owner = kind.create(opener);

            owner.init(context());

            assertEquals(kind.toString(), 1, opener.openCount);
            assertSame(kind.toString(), failure, retainedFailure(owner));
            assertSame(kind + " must not publish an owner when open throws",
                    null, field(owner, kind.laneField));
            assertFalse(kind + " clean unpublished failure must allow replacement",
                    booleanField(owner, "visionCleanupFailed"));

            RuntimeException replacementFailure =
                    new IllegalArgumentException(kind + " replacement setup failed");
            LaneProbe replacement = LaneProbe.failingSetup(replacementFailure, null);
            opener.openFailure = null;
            opener.lanes.addLast(replacement);
            retryFreshSelection(kind, owner);

            assertEquals(kind.toString(), 2, opener.openCount);
            assertSame(kind.toString(), replacementFailure, retainedFailure(owner));
            assertEquals(kind.toString(), 1, replacement.closeCount);
            owner.stop();
            assertEquals(kind.toString(), 1, replacement.closeCount);
        }
    }

    @Test
    public void pickerBuilderFailureOrNullFactoryNeverInvokesOpen() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException failure = new IllegalStateException(kind + " builder failed");
            BuilderProbe throwingBuilder = BuilderProbe.failing(failure);
            TeleOpTester throwingOwner = kind.createPicker(throwingBuilder.builder());

            throwingOwner.init(context());
            selectPickerName(kind, throwingOwner);

            assertEquals(kind.toString(), 1, throwingBuilder.applyCount);
            assertEquals(kind.toString(), 0, throwingBuilder.opener.openCount);
            assertSame(kind.toString(), failure, retainedFailure(throwingOwner));
            assertSame(kind.toString(), null, field(throwingOwner, kind.laneField));
            assertFalse(kind + " builder failure has no uncertain resource",
                    booleanField(throwingOwner, "visionCleanupFailed"));

            BuilderProbe nullBuilder = BuilderProbe.returningNull();
            TeleOpTester nullOwner = kind.createPicker(nullBuilder.builder());

            nullOwner.init(context());
            selectPickerName(kind, nullOwner);

            assertEquals(kind.toString(), 1, nullBuilder.applyCount);
            assertEquals(kind.toString(), 0, nullBuilder.opener.openCount);
            assertSame(kind.toString(), null, field(nullOwner, kind.laneField));
            assertFalse(kind + " null factory has no uncertain resource",
                    booleanField(nullOwner, "visionCleanupFailed"));
            assertTrue(kind.toString(), retainedFailure(nullOwner).getMessage().contains("null"));
        }
    }

    @Test
    public void factoryOpenReturningNullHasOneOpenAndNoOwnerToClose() {
        for (OwnerKind kind : OwnerKind.values()) {
            OpenProbe opener = OpenProbe.returningNull();
            TeleOpTester owner = kind.create(opener);

            owner.init(context());

            assertEquals(kind.toString(), 1, opener.openCount);
            assertSame(kind + " must not publish a null lane", null,
                    field(owner, kind.laneField));
            assertFalse(kind + " has no returned owner whose cleanup can be uncertain",
                    booleanField(owner, "visionCleanupFailed"));
            assertTrue(kind.toString(), retainedFailure(owner).getMessage().contains("returned null"));
        }
    }

    @Test
    public void nullFactoryDescriptionIsALegalTelemetryOmission() {
        for (OwnerKind kind : Arrays.asList(
                OwnerKind.CAMERA_MOUNT,
                OwnerKind.APRILTAG_LOCALIZATION
        )) {
            LaneProbe lane = LaneProbe.open(null);
            TeleOpTester owner = kind.create(OpenProbe.withDescription(null, lane));

            owner.init(context());

            assertTrue(kind.toString(), booleanField(owner, kind.readyField));
            assertSame(kind.toString(), null, field(owner, "activeVisionDescription"));
            assertEquals(kind.toString(), 0, lane.closeCount);
            owner.stop();
            assertEquals(kind.toString(), 1, lane.closeCount);
        }

        LaneProbe correctedLane = LaneProbe.open(null);
        OpenProbe correctedOpener = OpenProbe.withDescription(null, correctedLane);
        PinpointAprilTagCorrectedLocalizationTester.Config cfg =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        cfg.preferredVisionDeviceName = "vision";
        cfg.visionDeviceType = HardwareDevice.class;
        cfg.visionPickerTitle = "Vision";
        cfg.fixedTagLayout = new SimpleTagLayout();
        cfg.localization.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.LIMELIGHT_FIELD_POSE;
        TeleOpTester corrected = new PinpointAprilTagCorrectedLocalizationTester(
                cfg,
                ignored -> correctedOpener.factory()
        );

        corrected.init(context());

        assertEquals(1, correctedOpener.openCount);
        assertEquals(1, correctedLane.closeCount);
        assertTrue(retainedFailure(corrected).getMessage().contains("LIMELIGHT_FIELD_POSE"));
        assertFalse(retainedFailure(corrected).getMessage().contains("description"));
    }

    @Test
    public void throwingFactoryDescriptionClosesThePublishedLaneOnce() {
        for (OwnerKind kind : OwnerKind.values()) {
            RuntimeException descriptionFailure =
                    new IllegalStateException(kind + " description failed");
            LaneProbe lane = LaneProbe.open(null);
            TeleOpTester owner = kind.create(
                    OpenProbe.failingDescription(descriptionFailure, lane));

            owner.init(context());

            assertSame(kind.toString(), descriptionFailure, retainedFailure(owner));
            assertEquals(kind.toString(), 1, lane.closeCount);
            assertSame(kind.toString(), null, field(owner, kind.laneField));
        }
    }

    @Test
    public void nullReadinessIsAContractFailureForEveryVisionOwner() {
        for (OwnerKind kind : OwnerKind.values()) {
            LaneProbe lane = LaneProbe.open(null);
            lane.readiness = null;
            TeleOpTester owner;
            if (kind == OwnerKind.PINPOINT_CORRECTED) {
                owner = kind.create(new OpenProbe(LaneProbe.failingSetup(
                        new IllegalStateException("fixture stops before Pinpoint lookup"), null)));
                owner.init(context());
                setField(owner, kind.laneField, lane);
                setBooleanField(owner, kind.readyField, false);
                setField(owner, "visionFailure", null);
                setField(owner, "initError", null);
                invokePrivate(owner, "refreshVisionReadiness", new Class<?>[0]);
            } else {
                owner = kind.create(new OpenProbe(lane));
                owner.init(context());
            }

            assertEquals(kind.toString(), 1, lane.closeCount);
            assertSame(kind + " must detach a lane that violates readiness contract",
                    null, field(owner, kind.laneField));
            assertFalse(kind + " clean close permits a later picker selection",
                    booleanField(owner, "visionCleanupFailed"));
            RuntimeException failure = retainedFailure(owner);
            assertTrue(kind.toString(), failure.getMessage().contains("readiness"));
        }
    }

    @Test
    public void nullTagSensorClosesEveryPublishedOwnerExactlyOnce() {
        for (OwnerKind kind : OwnerKind.values()) {
            LaneProbe lane = LaneProbe.open(null);
            lane.returnNullSensor = true;
            OpenProbe opener = new OpenProbe(lane);
            TeleOpTester owner = kind.create(opener);

            owner.init(context());

            assertEquals(kind.toString(), 1, opener.openCount);
            assertEquals(kind.toString(), 1, lane.closeCount);
            assertSame(kind.toString(), null, field(owner, kind.laneField));
            assertTrue(kind.toString(), retainedFailure(owner).getMessage()
                    .toLowerCase(java.util.Locale.ROOT).contains("sensor"));
            owner.stop();
            assertEquals(kind + " detached owner must not close twice", 1, lane.closeCount);
        }
    }

    @Test
    public void nullCameraMountClosesAprilAndCorrectedOwnersExactlyOnce() {
        for (OwnerKind kind : Arrays.asList(
                OwnerKind.APRILTAG_LOCALIZATION,
                OwnerKind.PINPOINT_CORRECTED
        )) {
            LaneProbe lane = LaneProbe.open(null);
            lane.returnNullCameraMount = true;
            OpenProbe opener = new OpenProbe(lane);
            TeleOpTester owner = kind.create(opener);

            owner.init(context());

            assertEquals(kind.toString(), 1, opener.openCount);
            assertEquals(kind.toString(), 1, lane.closeCount);
            assertSame(kind.toString(), null, field(owner, kind.laneField));
            assertTrue(kind.toString(), retainedFailure(owner).getMessage()
                    .toLowerCase(java.util.Locale.ROOT).contains("mount"));
            owner.stop();
            assertEquals(kind + " detached owner must not close twice", 1, lane.closeCount);
        }
    }

    @Test
    public void pinpointPodOffsetAssistBlocksReplacementWhenFailedOwnerCannotClose() {
        PinpointPodOffsetCalibrator owner = newPodOwnerWithAssist();
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
        PinpointPodOffsetCalibrator owner = newPodOwnerWithAssist();
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
        PinpointPodOffsetCalibrator owner = newPodOwnerWithAssist();
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
        Function<String, AprilTagVisionLaneFactory> builder = ignored -> hardwareMap -> {
            openCalls[0]++;
            throw primary;
        };
        PinpointPodOffsetCalibrator owner = new PinpointPodOffsetCalibrator(cfg, builder);
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
        PinpointPodOffsetCalibrator owner = newPodOwnerWithAssist();
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
    public void pinpointPodOffsetFinalStopPreservesDriveFailureAndStillClosesVision() {
        PinpointPodOffsetCalibrator owner = newPodOwnerWithAssist();
        RuntimeException driveFailure = new IllegalStateException("drive stop failed");
        RuntimeException visionFailure = new IllegalArgumentException("vision close failed");
        PowerProbe failingOutput = new PowerProbe(driveFailure);
        PowerProbe unusedOutput = new PowerProbe(null);
        MecanumDrivebase drive = new MecanumDrivebase(
                failingOutput,
                unusedOutput,
                unusedOutput,
                unusedOutput,
                MecanumDrivebase.Config.defaults()
        );
        LaneProbe lane = LaneProbe.open(visionFailure);
        setField(owner, "drive", drive);
        setField(owner, "visionLane", lane);

        try {
            owner.stop();
            fail("Expected drive stop failure");
        } catch (RuntimeException actual) {
            assertSame(driveFailure, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertSame(visionFailure, actual.getSuppressed()[0]);
        }

        assertEquals(1, failingOutput.stopCount);
        assertEquals(1, lane.closeCount);
        assertSame(null, field(owner, "drive"));
        assertSame(null, field(owner, "visionLane"));

        owner.stop();
        assertEquals(1, failingOutput.stopCount);
        assertEquals(1, lane.closeCount);
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
        PINPOINT_CORRECTED("openSelectedVision", "visionLane", "ready");

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
            return create("vision", ignored -> opener.factory());
        }

        TeleOpTester createPicker(
                Function<String, AprilTagVisionLaneFactory> builder
        ) {
            return create(null, builder);
        }

        private TeleOpTester create(
                String preferredName,
                Function<String, AprilTagVisionLaneFactory> builder
        ) {
            switch (this) {
                case CAMERA_MOUNT:
                    CameraMountCalibrator.Config camera =
                            CameraMountCalibrator.Config.defaults();
                    camera.preferredVisionDeviceName = preferredName;
                    camera.visionDeviceType = HardwareDevice.class;
                    camera.visionPickerTitle = "Vision";
                    camera.fixedTagLayout = new SimpleTagLayout();
                    return new CameraMountCalibrator(camera, builder);
                case APRILTAG_LOCALIZATION:
                    AprilTagLocalizationTester.Config april =
                            AprilTagLocalizationTester.Config.defaults();
                    april.preferredVisionDeviceName = preferredName;
                    april.visionDeviceType = HardwareDevice.class;
                    april.visionPickerTitle = "Vision";
                    april.fixedTagLayout = new SimpleTagLayout();
                    return new AprilTagLocalizationTester(april, builder);
                case PINPOINT_CORRECTED:
                    PinpointAprilTagCorrectedLocalizationTester.Config corrected =
                            PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
                    corrected.preferredVisionDeviceName = preferredName;
                    corrected.visionDeviceType = HardwareDevice.class;
                    corrected.visionPickerTitle = "Vision";
                    corrected.localization =
                            FtcOdometryAprilTagLocalizationLane.Config.defaults();
                    corrected.fixedTagLayout = new SimpleTagLayout();
                    return new PinpointAprilTagCorrectedLocalizationTester(corrected, builder);
                default:
                    throw new AssertionError(this);
            }
        }
    }

    private static final class BuilderProbe {
        private final OpenProbe opener = new OpenProbe();
        private final RuntimeException failure;
        private final boolean returnNull;
        private int applyCount;

        private BuilderProbe(RuntimeException failure, boolean returnNull) {
            this.failure = failure;
            this.returnNull = returnNull;
        }

        static BuilderProbe failing(RuntimeException failure) {
            return new BuilderProbe(failure, false);
        }

        static BuilderProbe returningNull() {
            return new BuilderProbe(null, true);
        }

        Function<String, AprilTagVisionLaneFactory> builder() {
            return ignored -> {
                applyCount++;
                if (failure != null) {
                    throw failure;
                }
                return returnNull ? null : opener.factory();
            };
        }
    }

    private static final class OpenProbe {
        private final Deque<LaneProbe> lanes;
        private RuntimeException openFailure;
        private RuntimeException descriptionFailure;
        private String description = "test AprilTag vision";
        private boolean returnNull;
        private int openCount;

        OpenProbe(LaneProbe... lanes) {
            this.lanes = new ArrayDeque<>(Arrays.asList(lanes));
        }

        static OpenProbe failingOpen(RuntimeException openFailure) {
            OpenProbe probe = new OpenProbe();
            probe.openFailure = openFailure;
            return probe;
        }

        static OpenProbe returningNull() {
            OpenProbe probe = new OpenProbe();
            probe.returnNull = true;
            return probe;
        }

        static OpenProbe withDescription(String description, LaneProbe... lanes) {
            OpenProbe probe = new OpenProbe(lanes);
            probe.description = description;
            return probe;
        }

        static OpenProbe failingDescription(
                RuntimeException descriptionFailure,
                LaneProbe... lanes
        ) {
            OpenProbe probe = new OpenProbe(lanes);
            probe.descriptionFailure = descriptionFailure;
            return probe;
        }

        AprilTagVisionLaneFactory factory() {
            return new AprilTagVisionLaneFactory() {
                @Override
                public AprilTagVisionLane open(HardwareMap hardwareMap) {
                    openCount++;
                    if (openFailure != null) {
                        throw openFailure;
                    }
                    if (returnNull) {
                        return null;
                    }
                    LaneProbe lane = lanes.pollFirst();
                    if (lane == null) {
                        throw new IllegalStateException("No queued test lane");
                    }
                    return lane;
                }

                @Override
                public String description() {
                    if (descriptionFailure != null) {
                        throw descriptionFailure;
                    }
                    return description;
                }
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
        private boolean returnNullSensor;
        private boolean returnNullCameraMount;

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
            if (returnNullSensor) {
                return null;
            }
            return clock -> AprilTagDetections.none();
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            return returnNullCameraMount ? null : CameraMountConfig.identity();
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

    private static final class PowerProbe implements PowerOutput {
        private final RuntimeException stopFailure;
        private double commandedPower;
        private int stopCount;

        private PowerProbe(RuntimeException stopFailure) {
            this.stopFailure = stopFailure;
        }

        @Override
        public void setPower(double power) {
            stopCount++;
            if (stopFailure != null) {
                throw stopFailure;
            }
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }
    }

    private static PinpointPodOffsetCalibrator newPodOwnerWithAssist() {
        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        Function<String, AprilTagVisionLaneFactory> builder = ignored -> hardwareMap -> {
            throw new AssertionError("test did not provide an AprilTag lane");
        };
        return new PinpointPodOffsetCalibrator(cfg, builder);
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

    private static void retryFreshSelection(OwnerKind kind, TeleOpTester owner) {
        if (kind == OwnerKind.PINPOINT_CORRECTED) {
            setField(owner, "selectedVisionDeviceName", "vision");
            invokeEnsure(kind, owner);
            return;
        }
        invokePrivate(
                owner,
                "preparePickerSelection",
                new Class<?>[]{String.class},
                "vision"
        );
    }

    private static void selectPickerName(OwnerKind kind, TeleOpTester owner) {
        retryFreshSelection(kind, owner);
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
