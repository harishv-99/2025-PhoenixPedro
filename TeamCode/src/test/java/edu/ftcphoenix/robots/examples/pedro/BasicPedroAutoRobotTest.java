package edu.ftcphoenix.robots.examples.pedro;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetStatus;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the basic Pedro declarations through the framework-owned FTC lifecycle. */
public final class BasicPedroAutoRobotTest {

    @Test
    public void initDoesNotActuateAndStartPreservesRequiredOwnershipOrder() {
        List<String> events = new ArrayList<String>();
        RecordingOpMode mode = recordingMode(events);
        mode.telemetry = inertTelemetry();
        mode.runtimeSec = 10.0;

        mode.init();
        mode.runtimeSec = 10.1;
        mode.init_loop();
        assertTrue(events.isEmpty());

        mode.start();
        assertEquals(
                Arrays.asList(
                        "startPose",
                        "localization(cycle=3,dt=0.00)",
                        "drive.update(cycle=3)",
                        "task.start(cycle=3)",
                        "task.update(cycle=3)",
                        "plant.update(cycle=3)"
                ),
                events
        );

        mode.runtimeSec = 10.35;
        mode.loop();
        assertEquals(
                Arrays.asList(
                        "startPose",
                        "localization(cycle=3,dt=0.00)",
                        "drive.update(cycle=3)",
                        "task.start(cycle=3)",
                        "task.update(cycle=3)",
                        "plant.update(cycle=3)",
                        "localization(cycle=4,dt=0.25)",
                        "drive.update(cycle=4)",
                        "task.update(cycle=4)",
                        "plant.update(cycle=4)"
                ),
                events
        );
        assertEquals(TaskOutcome.NOT_DONE, mode.robot.rootOutcome());
    }

    @Test
    public void stopUsesRunnerOutputServiceOrderAndAttemptsEveryCleanup() {
        List<String> events = new ArrayList<String>();
        RuntimeException taskFailure = new RuntimeException("task cleanup failed");
        RuntimeException mechanismFailure = new RuntimeException("mechanism stop failed");
        RuntimeException driveFailure = new RuntimeException("drive stop failed");
        RecordingPlant plant = new RecordingPlant(events);
        plant.stopFailure = mechanismFailure;
        RecordingTask task = new RecordingTask(events);
        task.cancelFailure = taskFailure;
        RecordingDrive drive = new RecordingDrive(events);
        drive.stopFailure = driveFailure;
        RecordingOpMode mode = new RecordingOpMode(program -> new BasicPedroAutoRobot(
                program,
                new RecordingLocalization(events),
                drive,
                () -> events.add("startPose"),
                new BasicPedroAutoMechanism(plant, 0.7),
                task
        ));
        mode.telemetry = inertTelemetry();
        mode.init();
        mode.start();
        events.clear();

        try {
            mode.stop();
            fail("expected cleanup failure");
        } catch (RuntimeException failure) {
            assertSame(taskFailure, failure);
            assertEquals(2, failure.getSuppressed().length);
            assertSame(mechanismFailure, failure.getSuppressed()[0]);
            assertSame(driveFailure, failure.getSuppressed()[1]);
        }

        assertEquals(
                Arrays.asList("task.cancel", "plant.stop", "drive.stop"),
                events
        );
        assertEquals(0.0, plant.target.get(), 0.0);
        mode.stop();
        assertEquals(3, events.size());
    }

    @Test
    public void activeFailureRemainsPrimaryAndProgramPerformsCompleteCleanup() {
        List<String> events = new ArrayList<String>();
        RuntimeException updateFailure = new RuntimeException("plant update failed");
        RuntimeException taskFailure = new RuntimeException("task cleanup failed");
        RuntimeException mechanismFailure = new RuntimeException("mechanism stop failed");
        RuntimeException driveFailure = new RuntimeException("drive stop failed");
        RecordingPlant plant = new RecordingPlant(events);
        plant.stopFailure = mechanismFailure;
        RecordingTask task = new RecordingTask(events);
        task.cancelFailure = taskFailure;
        RecordingDrive drive = new RecordingDrive(events);
        drive.stopFailure = driveFailure;
        RecordingOpMode mode = new RecordingOpMode(program -> new BasicPedroAutoRobot(
                program,
                new RecordingLocalization(events),
                drive,
                () -> events.add("startPose"),
                new BasicPedroAutoMechanism(plant, 0.7),
                task
        ));
        mode.telemetry = inertTelemetry();
        mode.init();
        mode.start();
        events.clear();
        plant.updateFailure = updateFailure;
        mode.runtimeSec = 0.02;

        try {
            mode.loop();
            fail("expected update failure");
        } catch (RuntimeException failure) {
            assertSame(updateFailure, failure);
            assertEquals(3, failure.getSuppressed().length);
            assertSame(taskFailure, failure.getSuppressed()[0]);
            assertSame(mechanismFailure, failure.getSuppressed()[1]);
            assertSame(driveFailure, failure.getSuppressed()[2]);
        }

        assertEquals(
                Arrays.asList(
                        "localization(cycle=3,dt=0.02)",
                        "drive.update(cycle=3)",
                        "task.update(cycle=3)",
                        "plant.update(cycle=3)",
                        "task.cancel",
                        "plant.stop",
                        "drive.stop"
                ),
                events
        );
    }

    @Test
    public void failureAfterServiceRegistrationStopsAlreadyOwnedResources() {
        List<String> events = new ArrayList<String>();
        RuntimeException constructionFailure = new RuntimeException("later construction failed");
        RecordingDrive drive = new RecordingDrive(events);
        RecordingOpMode mode = new RecordingOpMode(program -> {
            new BasicPedroAutoRobot(
                    program,
                    new RecordingLocalization(events),
                    drive,
                    () -> events.add("startPose"),
                    new BasicPedroAutoMechanism(new RecordingPlant(events), 0.7),
                    new RecordingTask(events)
            );
            throw constructionFailure;
        });
        mode.telemetry = inertTelemetry();

        try {
            mode.init();
            fail("expected construction failure");
        } catch (RuntimeException failure) {
            assertSame(constructionFailure, failure);
        }

        assertEquals(Arrays.asList("plant.stop", "drive.stop"), events);
    }

    /** Create the same fake-backed declaration for the separate FTC host lifecycle test. */
    public static BasicPedroAutoRobot newRecordingRobot(RobotProgram program,
                                                         List<String> events) {
        RecordingPlant plant = new RecordingPlant(events);
        return new BasicPedroAutoRobot(
                program,
                new RecordingLocalization(events),
                new RecordingDrive(events),
                () -> events.add("startPose"),
                new BasicPedroAutoMechanism(plant, 0.7),
                new RecordingTask(events)
        );
    }

    private static RecordingOpMode recordingMode(List<String> events) {
        return new RecordingOpMode(program -> newRecordingRobot(program, events));
    }

    private static final class RecordingOpMode extends FtcRobotOpMode {
        private final Function<RobotProgram, BasicPedroAutoRobot> factory;
        BasicPedroAutoRobot robot;
        double runtimeSec;

        RecordingOpMode(Function<RobotProgram, BasicPedroAutoRobot> factory) {
            this.factory = factory;
        }

        @Override
        protected void configure(RobotProgram program) {
            robot = factory.apply(program);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final class RecordingLocalization implements AbsolutePoseEstimator {
        private final List<String> events;

        RecordingLocalization(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            events.add(String.format(
                    "localization(cycle=%d,dt=%.2f)",
                    clock.cycle(),
                    clock.dtSec()
            ));
        }

        @Override
        public PoseEstimate getEstimate() {
            return PoseEstimate.noPose(LoopTimestamp.unavailable());
        }
    }

    private static final class RecordingDrive implements DriveCommandSink {
        private final List<String> events;
        RuntimeException stopFailure;

        RecordingDrive(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            events.add("drive.update(cycle=" + clock.cycle() + ")");
        }

        @Override
        public void drive(DriveSignal signal) {
            events.add("drive.command");
        }

        @Override
        public void stop() {
            events.add("drive.stop");
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingPlant implements Plant {
        private final List<String> events;
        final ScalarTarget target = ScalarTarget.create(0.0);
        RuntimeException updateFailure;
        RuntimeException stopFailure;

        RecordingPlant(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            events.add("plant.update(cycle=" + clock.cycle() + ")");
            if (updateFailure != null) {
                throw updateFailure;
            }
        }

        @Override
        public double getRequestedTarget() {
            return target.get();
        }

        @Override
        public double getAppliedTarget() {
            return target.get();
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasCommandTarget() {
            return true;
        }

        @Override
        public ScalarTarget commandTarget() {
            return target;
        }

        @Override
        public void stop() {
            events.add("plant.stop");
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingTask implements Task {
        private final List<String> events;
        private boolean started;
        private boolean cancelled;
        RuntimeException cancelFailure;

        RecordingTask(List<String> events) {
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            events.add("task.start(cycle=" + clock.cycle() + ")");
        }

        @Override
        public void update(LoopClock clock) {
            events.add("task.update(cycle=" + clock.cycle() + ")");
        }

        @Override
        public void cancel() {
            if (!started || cancelled) {
                return;
            }
            cancelled = true;
            events.add("task.cancel");
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            return cancelled;
        }

        @Override
        public TaskOutcome getOutcome() {
            return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }

    private static Telemetry inertTelemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> defaultValue(method.getReturnType())
        );
    }

    private static Object defaultValue(Class<?> returnType) {
        if (returnType == boolean.class) {
            return true;
        }
        if (returnType == byte.class) {
            return (byte) 0;
        }
        if (returnType == short.class) {
            return (short) 0;
        }
        if (returnType == int.class) {
            return 0;
        }
        if (returnType == long.class) {
            return 0L;
        }
        if (returnType == float.class) {
            return 0.0f;
        }
        if (returnType == double.class) {
            return 0.0;
        }
        if (returnType == char.class) {
            return '\0';
        }
        return null;
    }
}
