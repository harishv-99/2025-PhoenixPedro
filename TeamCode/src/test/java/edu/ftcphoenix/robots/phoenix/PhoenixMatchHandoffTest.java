package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.FtcAutoToTeleOpHandoff;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Host-side checks for Phoenix's narrow final-Auto-pose/alliance handoff. */
public final class PhoenixMatchHandoffTest {

    @Before
    public void clearBeforeTest() {
        PhoenixMatchHandoff.clear();
    }

    @After
    public void clearAfterTest() {
        PhoenixMatchHandoff.clear();
    }

    @Test
    public void validAutoPoseAndAllianceAreForwardedExactlyOnce() {
        TestAuto auto = new TestAuto();
        TestTeleOp teleOp = new TestTeleOp();
        RecordingMatchReceiver receiver = new RecordingMatchReceiver();
        PoseEstimate estimate = estimate(12.5, -7.25, 1.75, true);

        PhoenixMatchHandoff.publishFromAuto(auto, estimate, PhoenixAlliance.BLUE);

        assertEquals(
                PhoenixMatchHandoff.RestoreResult.RESTORED,
                PhoenixMatchHandoff.restoreForTeleOp(teleOp, receiver)
        );
        assertEquals(1, receiver.calls);
        assertEquals(new Pose2d(12.5, -7.25, 1.75), receiver.lastPose);
        assertEquals(PhoenixAlliance.BLUE, receiver.lastAlliance);

        assertEquals(
                PhoenixMatchHandoff.RestoreResult.ALREADY_CONSUMED,
                PhoenixMatchHandoff.restoreForTeleOp(new TestTeleOp(), receiver)
        );
        assertEquals(1, receiver.calls);
    }

    @Test
    public void missingSnapshotLeavesReceiverUntouched() {
        RecordingMatchReceiver receiver = new RecordingMatchReceiver();

        assertEquals(
                PhoenixMatchHandoff.RestoreResult.MISSING,
                PhoenixMatchHandoff.restoreForTeleOp(new TestTeleOp(), receiver)
        );
        assertEquals(0, receiver.calls);
    }

    @Test
    public void everyCarrierStatusMapsToOneRobotFacingResult() {
        assertEquals(
                PhoenixMatchHandoff.RestoreResult.RESTORED,
                PhoenixMatchHandoff.restoreResultFor(
                        FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED
                )
        );
        assertEquals(
                PhoenixMatchHandoff.RestoreResult.MISSING,
                PhoenixMatchHandoff.restoreResultFor(
                        FtcAutoToTeleOpHandoff.ConsumeStatus.MISSING
                )
        );
        assertEquals(
                PhoenixMatchHandoff.RestoreResult.STALE,
                PhoenixMatchHandoff.restoreResultFor(
                        FtcAutoToTeleOpHandoff.ConsumeStatus.STALE
                )
        );
        assertEquals(
                PhoenixMatchHandoff.RestoreResult.ALREADY_CONSUMED,
                PhoenixMatchHandoff.restoreResultFor(
                        FtcAutoToTeleOpHandoff.ConsumeStatus.ALREADY_CONSUMED
                )
        );
    }

    @Test
    public void invalidEstimateIsRejectedBeforePublication() {
        assertPublishRejected(
                PoseEstimate.noPose(LoopTimestamp.unavailable()),
                "hasPose=false"
        );
        assertPublishRejected(
                estimate(Double.NaN, 0.0, 0.0, true),
                "finite xInches"
        );
        assertPublishRejected(
                estimate(0.0, Double.POSITIVE_INFINITY, 0.0, true),
                "finite xInches"
        );
        assertPublishRejected(
                estimate(0.0, 0.0, Double.NEGATIVE_INFINITY, true),
                "finite xInches"
        );

        RecordingMatchReceiver receiver = new RecordingMatchReceiver();
        assertEquals(
                PhoenixMatchHandoff.RestoreResult.MISSING,
                PhoenixMatchHandoff.restoreForTeleOp(new TestTeleOp(), receiver)
        );
        assertEquals(0, receiver.calls);
    }

    @Test
    public void nullArgumentsFailBeforeChangingTheOpenCycle() {
        PoseEstimate valid = estimate(1.0, 2.0, 3.0, true);

        try {
            PhoenixMatchHandoff.publishFromAuto(null, valid, PhoenixAlliance.RED);
            fail("Expected a null Auto OpMode to be rejected");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("Auto OpMode"));
        }
        try {
            PhoenixMatchHandoff.publishFromAuto(
                    new TestAuto(),
                    null,
                    PhoenixAlliance.RED
            );
            fail("Expected a null final estimate to be rejected");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("pose estimate"));
        }
        try {
            PhoenixMatchHandoff.publishFromAuto(new TestAuto(), valid, null);
            fail("Expected a null frozen alliance to be rejected");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("frozen Auto alliance"));
        }
        try {
            PhoenixMatchHandoff.restoreForTeleOp(
                    null,
                    (fieldToRobotPose, alliance) -> {
                    }
            );
            fail("Expected a null TeleOp OpMode to be rejected");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("TeleOp OpMode"));
        }

        PhoenixMatchHandoff.publishFromAuto(
                new TestAuto(),
                valid,
                PhoenixAlliance.RED
        );
        RecordingMatchReceiver receiver = new RecordingMatchReceiver();
        assertEquals(
                PhoenixMatchHandoff.RestoreResult.RESTORED,
                PhoenixMatchHandoff.restoreForTeleOp(new TestTeleOp(), receiver)
        );
        assertEquals(1, receiver.calls);
    }

    @Test
    public void nullReceiverIsRejectedWithoutConsumingTheSnapshot() {
        PhoenixMatchHandoff.publishFromAuto(
                new TestAuto(),
                estimate(1.0, 2.0, 3.0, true),
                PhoenixAlliance.BLUE
        );

        try {
            PhoenixMatchHandoff.restoreForTeleOp(
                    new TestTeleOp(),
                    (PhoenixMatchHandoff.MatchReceiver) null
            );
            fail("Expected a missing receiver to be rejected");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("match receiver"));
        }

        RecordingMatchReceiver receiver = new RecordingMatchReceiver();
        assertEquals(
                PhoenixMatchHandoff.RestoreResult.RESTORED,
                PhoenixMatchHandoff.restoreForTeleOp(new TestTeleOp(), receiver)
        );
        assertEquals(1, receiver.calls);
    }

    @Test
    public void publicSurfaceKeepsCarrierAndSnapshotPrivate() throws Exception {
        assertTrue(Modifier.isPublic(PhoenixMatchHandoff.class.getModifiers()));
        assertTrue(Modifier.isFinal(PhoenixMatchHandoff.class.getModifiers()));

        Constructor<?>[] constructors = PhoenixMatchHandoff.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        assertNotNull(PhoenixMatchHandoff.class.getMethod("clear"));
        assertNotNull(PhoenixMatchHandoff.class.getMethod(
                "publishFromAuto",
                OpMode.class,
                PoseEstimate.class,
                PhoenixAlliance.class
        ));
        assertNotNull(PhoenixMatchHandoff.class.getMethod(
                "restoreForTeleOp",
                OpMode.class,
                PhoenixRobot.class,
                Consumer.class
        ));

        List<String> publicMethods = new ArrayList<>();
        for (Method method : PhoenixMatchHandoff.class.getDeclaredMethods()) {
            if (!Modifier.isPublic(method.getModifiers())) {
                continue;
            }
            publicMethods.add(method.getName());
            assertFalse(
                    "Public Phoenix handoff methods must not expose the generic carrier",
                    method.toGenericString().contains("FtcAutoToTeleOpHandoff")
            );
            assertFalse(
                    "Public Phoenix handoff methods must not expose its private snapshot",
                    method.toGenericString().contains("MatchSnapshot")
            );
        }
        publicMethods.sort(String::compareTo);
        assertEquals(
                Arrays.asList("clear", "publishFromAuto", "restoreForTeleOp"),
                publicMethods
        );

        assertTrue(Modifier.isPublic(
                PhoenixMatchHandoff.RestoreResult.class.getModifiers()
        ));
        assertFalse(Modifier.isPublic(
                PhoenixMatchHandoff.MatchReceiver.class.getModifiers()
        ));
    }

    @Test
    public void robotPoseRestoreLifecycleRequiresInitializedPreStartTeleOp() {
        PhoenixRobot.TeleOpPoseRestoreLifecycle lifecycle =
                new PhoenixRobot.TeleOpPoseRestoreLifecycle();
        RecordingPoseResetter resetter = new RecordingPoseResetter();
        Pose2d expected = new Pose2d(9.0, -4.0, 0.75);

        try {
            lifecycle.restore(expected);
            fail("Expected restore before TeleOp initialization to fail");
        } catch (IllegalStateException expectedFailure) {
            assertTrue(expectedFailure.getMessage().contains(
                    "declareTeleOp(program, profile, gamepad1, gamepad2, eligibleScoringTagIds)"
            ));
        }

        lifecycle.initialize(resetter);
        lifecycle.restore(expected);
        assertEquals(1, resetter.calls);
        assertEquals(expected, resetter.lastPose);

        lifecycle.markStartBoundary();
        try {
            lifecycle.restore(new Pose2d(1.0, 2.0, 3.0));
            fail("Expected restore after FTC START to fail");
        } catch (IllegalStateException expectedFailure) {
            assertTrue(expectedFailure.getMessage().contains("after FTC START"));
        }
        assertEquals(1, resetter.calls);

        lifecycle.clear();
        try {
            lifecycle.restore(expected);
            fail("Expected restore after robot stop to fail");
        } catch (IllegalStateException expectedFailure) {
            assertTrue(expectedFailure.getMessage().contains(
                    "declareTeleOp(program, profile, gamepad1, gamepad2, eligibleScoringTagIds)"
            ));
        }
        assertEquals(1, resetter.calls);
    }

    @Test
    public void robotPoseRestoreLifecycleDefendsAgainstInvalidPose() {
        PhoenixRobot.TeleOpPoseRestoreLifecycle lifecycle =
                new PhoenixRobot.TeleOpPoseRestoreLifecycle();
        RecordingPoseResetter resetter = new RecordingPoseResetter();
        lifecycle.initialize(resetter);

        try {
            lifecycle.restore(new Pose2d(0.0, Double.NaN, 0.0));
            fail("Expected a non-finite pose to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("finite xInches"));
        }
        assertEquals(0, resetter.calls);
    }

    private static void assertPublishRejected(PoseEstimate estimate, String messageFragment) {
        try {
            PhoenixMatchHandoff.publishFromAuto(
                    new TestAuto(),
                    estimate,
                    PhoenixAlliance.RED
            );
            fail("Expected invalid final Auto pose to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messageFragment));
        }
    }

    private static PoseEstimate estimate(double xInches,
                                         double yInches,
                                         double headingRad,
                                         boolean hasPose) {
        return new PoseEstimate(
                new Pose3d(
                        xInches,
                        yInches,
                        4.0,
                        headingRad,
                        0.25,
                        -0.5
                ),
                hasPose,
                0.9,
                LoopTimestamp.unavailable()
        );
    }

    private static final class RecordingMatchReceiver
            implements PhoenixMatchHandoff.MatchReceiver {
        private int calls;
        private Pose2d lastPose;
        private PhoenixAlliance lastAlliance;

        @Override
        public void restore(Pose2d fieldToRobotPose, PhoenixAlliance alliance) {
            calls++;
            lastPose = fieldToRobotPose;
            lastAlliance = alliance;
        }
    }

    private static final class RecordingPoseResetter implements PoseResetter {
        private int calls;
        private Pose2d lastPose;

        @Override
        public void setPose(Pose2d pose) {
            calls++;
            lastPose = pose;
        }
    }

    private static final class TestAuto extends OpMode {
        @Override
        public void init() {
        }

        @Override
        public void loop() {
        }
    }

    private static final class TestTeleOp extends OpMode {
        @Override
        public void init() {
        }

        @Override
        public void loop() {
        }
    }
}
