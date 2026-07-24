package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix TeleOp's one-shot restore and initialization-failure cleanup boundary. */
public final class PhoenixTeleOpTest {

    @Before
    public void clearHandoffBeforeTest() {
        PhoenixMatchHandoff.clear();
    }

    @After
    public void clearHandoffAfterTest() {
        PhoenixMatchHandoff.clear();
    }

    @Test
    public void restoreFailureStopsInitializedRobotAndPreservesCleanupFailure()
            throws Exception {
        RuntimeException restoreFailure = new IllegalStateException("controlled restore failure");
        RuntimeException cleanupFailure = new IllegalStateException("controlled cleanup failure");
        PhoenixMatchHandoff.publishFromAuto(
                new EmptyOpMode(),
                new PoseEstimate(
                        new Pose3d(14.0, -6.0, 0.0, 0.75, 0.0, 0.0),
                        true,
                        1.0,
                        LoopTimestamp.unavailable()
                )
        );

        PhoenixRobot robot = uninitializedRobot();
        initializeRestoreLifecycle(
                robot,
                new PoseResetter() {
                    @Override
                    public void setPose(Pose2d pose) {
                        throw restoreFailure;
                    }
                }
        );
        setField(
                PhoenixRobot.class,
                robot,
                "autonomousDrive",
                new DriveCommandSink() {
                    @Override
                    public void drive(DriveSignal signal) {
                        // No hardware in this lifecycle regression.
                    }

                    @Override
                    public void stop() {
                        throw cleanupFailure;
                    }
                }
        );

        try {
            PhoenixTeleOp.restoreInitializedRobotOrStop(new EmptyOpMode(), robot);
            fail("expected restore failure");
        } catch (RuntimeException failure) {
            assertSame(restoreFailure, failure);
            assertEquals(1, failure.getSuppressed().length);
            assertSame(cleanupFailure, failure.getSuppressed()[0]);
        }

        assertNull(getField(PhoenixRobot.class, robot, "autonomousDrive"));
        Object lifecycle = getField(PhoenixRobot.class, robot, "teleOpPoseRestore");
        assertNull(getField(lifecycle.getClass(), lifecycle, "poseResetter"));
    }

    @Test
    public void missingHandoffFallbackSharesTheCommittedInitReadinessFrame() {
        List<String> pendingEntries = new ArrayList<>();
        List<List<String>> committedFrames = new ArrayList<>();
        Telemetry recordingTelemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if ("addData".equals(method.getName()) || "addLine".equals(method.getName())) {
                        pendingEntries.add(java.util.Arrays.toString(args));
                    } else if ("update".equals(method.getName())) {
                        committedFrames.add(new ArrayList<>(pendingEntries));
                        pendingEntries.clear();
                    }
                    Class<?> returnType = method.getReturnType();
                    if (returnType == boolean.class) {
                        return true;
                    }
                    if (returnType == int.class) {
                        return 0;
                    }
                    return null;
                }
        );

        recordingTelemetry.addLine("ordinary controls and pose-assist readiness");
        PhoenixTeleOp.publishAutoHandoffTelemetry(
                recordingTelemetry,
                PhoenixMatchHandoff.RestoreResult.MISSING
        );

        assertEquals(1, committedFrames.size());
        String initFrame = committedFrames.get(0).toString();
        assertTrue(initFrame.contains("ordinary controls and pose-assist readiness"));
        assertTrue(initFrame.contains("teleop.autoHandoff"));
        assertTrue(initFrame.contains("MISSING"));
        assertTrue(initFrame.contains("normally initialized TeleOp pose"));
    }

    private static PhoenixRobot uninitializedRobot() {
        return new PhoenixRobot(
                new HardwareMap(null, null),
                inertTelemetry(),
                new Gamepad(),
                new Gamepad(),
                PhoenixProfile.current()
        );
    }

    private static void initializeRestoreLifecycle(PhoenixRobot robot,
                                                   PoseResetter poseResetter)
            throws Exception {
        Object lifecycle = getField(PhoenixRobot.class, robot, "teleOpPoseRestore");
        Method initialize = lifecycle.getClass().getDeclaredMethod(
                "initialize",
                PoseResetter.class
        );
        initialize.setAccessible(true);
        initialize.invoke(lifecycle, poseResetter);
    }

    private static void setField(Class<?> owner,
                                 Object target,
                                 String name,
                                 Object value) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static Object getField(Class<?> owner,
                                   Object target,
                                   String name) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(target);
    }

    private static Telemetry inertTelemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    Class<?> returnType = method.getReturnType();
                    if (returnType == boolean.class) {
                        return true;
                    }
                    if (returnType == int.class) {
                        return 0;
                    }
                    return null;
                }
        );
    }

    private static final class EmptyOpMode extends OpMode {
        @Override
        public void init() {
        }

        @Override
        public void loop() {
        }
    }
}
