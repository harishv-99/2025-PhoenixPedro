package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.input.binding.BindingRegistrar;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskBindings;
import edu.ftcphoenix.fw.task.TaskOutcome;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the managed FTC robot program's fixed-order, fail-stop lifecycle. */
public final class FtcRobotOpModeTest {

    @Test
    public void initIsPresenterOnlyAndAdvancesOneSharedClockExactlyOncePerFrame() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service = new RecordingService("service", events);
        RecordingOutput output = new RecordingOutput("output", events);
        RecordingDriveSource driveSource = new RecordingDriveSource(events);
        RecordingDriveSink driveSink = new RecordingDriveSink(events);
        RecordingTask root = new RecordingTask("root", events);
        int[] bindingSamples = {0};
        int[] bindingActions = {0};
        List<ClockSnapshot> presenterFrames = new ArrayList<ClockSnapshot>();
        List<LoopClock> presenterClocks = new ArrayList<LoopClock>();

        mode.configureAction = program -> {
            program.service(service);
            program.bindings().whileHigh(clock -> {
                bindingSamples[0]++;
                return true;
            }, () -> bindingActions[0]++);
            program.rootTask(root);
            program.output(output);
            program.drive(driveSource, driveSink);
            program.presenter((clock, destination) -> {
                assertSame(telemetry.proxy(), destination);
                presenterClocks.add(clock);
                presenterFrames.add(ClockSnapshot.of(clock));
                events.add("presenter");
            });
        };

        mode.runtimeSec = 10.0;
        mode.init();
        mode.runtimeSec = 10.25;
        mode.init_loop();

        assertEquals(1, mode.configureCalls);
        assertEquals(2, presenterFrames.size());
        assertEquals(new ClockSnapshot(1L, 10.0, 0.0), presenterFrames.get(0));
        assertEquals(new ClockSnapshot(2L, 10.25, 0.25), presenterFrames.get(1));
        assertSame(presenterClocks.get(0), presenterClocks.get(1));
        assertEquals(2, telemetry.updateCalls);

        assertEquals(0, service.startCalls);
        assertEquals(0, service.updateCalls);
        assertEquals(0, output.updateCalls);
        assertEquals(0, driveSink.updateCalls);
        assertEquals(0, driveSource.sampleCalls);
        assertEquals(0, driveSink.driveCalls);
        assertEquals(0, root.startCalls);
        assertEquals(0, root.updateCalls);
        assertEquals(0, bindingSamples[0]);
        assertEquals(0, bindingActions[0]);

        mode.stop();
    }

    @Test
    public void startResetsClockStartsServicesAndRootThenRealizesExactStartOutputs() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service1 = new RecordingService("service1", events);
        RecordingService service2 = new RecordingService("service2", events);
        RecordingTask root = new RecordingTask("root", events);
        RecordingOutput output1 = new RecordingOutput("output1", events);
        RecordingOutput output2 = new RecordingOutput("output2", events);
        int[] requestedValue = {0};
        int[] presenterCalls = {0};

        root.updateAction = () -> requestedValue[0] = 7;
        output1.updateAction = () -> assertEquals(7, requestedValue[0]);
        mode.configureAction = program -> {
            program.service(service1);
            program.service(service2);
            program.rootTask(root);
            program.output(output1);
            program.output(output2);
            program.presenter((clock, destination) -> presenterCalls[0]++);
        };

        mode.runtimeSec = 3.0;
        mode.init();
        int commitsAfterInit = telemetry.updateCalls;
        events.clear();

        mode.runtimeSec = 20.0;
        mode.start();

        assertEquals(
                Arrays.asList(
                        "service1.start",
                        "service2.start",
                        "root.start",
                        "root.update",
                        "output1.update",
                        "output2.update"
                ),
                events
        );
        assertEquals(new ClockSnapshot(2L, 20.0, 0.0), ClockSnapshot.of(service1.startClock));
        assertSame(service1.startClock, service2.startClock);
        assertSame(service1.startClock, root.startClock);
        assertSame(service1.startClock, root.updateClock);
        assertSame(service1.startClock, output1.updateClock);
        assertSame(service1.startClock, output2.updateClock);
        assertEquals(1, root.startCalls);
        assertEquals(1, root.updateCalls);
        assertEquals(1, output1.updateCalls);
        assertEquals(1, output2.updateCalls);
        assertEquals(commitsAfterInit, telemetry.updateCalls);
        assertEquals(1, presenterCalls[0]);

        mode.stop();
    }

    @Test
    public void activeLoopUsesFixedOrderAndCommitsOneTelemetryFrame() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service1 = new RecordingService("service1", events);
        RecordingService service2 = new RecordingService("service2", events);
        RecordingTask root = new RecordingTask("root", events);
        RecordingOutput output1 = new RecordingOutput("output1", events);
        RecordingOutput output2 = new RecordingOutput("output2", events);
        RecordingDriveSource driveSource = new RecordingDriveSource(events);
        RecordingDriveSink driveSink = new RecordingDriveSink(events);

        mode.configureAction = program -> {
            program.service(service1);
            program.service(service2);
            program.bindings().whileHigh(
                    BooleanSource.constant(true),
                    () -> events.add("binding")
            );
            program.rootTask(root);
            program.output(output1);
            program.drive(driveSource, driveSink);
            program.output(output2);
            program.presenter((clock, destination) -> events.add("presenter1"));
            program.presenter((clock, destination) -> events.add("presenter2"));
        };

        mode.runtimeSec = 1.0;
        mode.init();
        mode.runtimeSec = 2.0;
        mode.start();
        int commitsBeforeLoop = telemetry.updateCalls;
        events.clear();

        mode.runtimeSec = 2.5;
        mode.loop();

        assertEquals(
                Arrays.asList(
                        "service1.update",
                        "service2.update",
                        "binding",
                        "root.update",
                        "output1.update",
                        "drive.update",
                        "drive.sample",
                        "drive.write",
                        "output2.update",
                        "presenter1",
                        "presenter2",
                        "telemetry.update"
                ),
                events
        );
        assertEquals(commitsBeforeLoop + 1, telemetry.updateCalls);
        assertEquals(new ClockSnapshot(3L, 2.5, 0.5), ClockSnapshot.of(service1.updateClock));
        assertSame(service1.updateClock, service2.updateClock);
        assertSame(service1.updateClock, root.updateClock);
        assertSame(service1.updateClock, output1.updateClock);
        assertSame(service1.updateClock, driveSink.updateClock);
        assertSame(service1.updateClock, driveSource.sampleClock);
        assertSame(service1.updateClock, output2.updateClock);

        mode.stop();
    }

    @Test
    public void bindingEnqueuedFreshTasksReachTheSameCycleOutputPhase() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        boolean[] pressed = {false};
        int[] requested = {0};
        List<RecordingTask> created = new ArrayList<RecordingTask>();
        List<Integer> realizedRequests = new ArrayList<Integer>();
        List<Long> realizedCycles = new ArrayList<Long>();
        RecordingOutput output = new RecordingOutput("output", events);

        output.updateAction = () -> {
            realizedRequests.add(requested[0]);
            realizedCycles.add(output.updateClock.cycle());
            events.add("realized:" + requested[0]);
        };
        mode.configureAction = program -> {
            program.taskBindings().onRise(clock -> pressed[0], () -> {
                int id = created.size() + 1;
                events.add("factory:" + id);
                RecordingTask task = new RecordingTask("task" + id, events);
                task.completeOnUpdate = true;
                task.updateAction = () -> requested[0] = id;
                created.add(task);
                return task;
            });
            program.output(output);
        };

        mode.init();
        mode.start();
        mode.runtimeSec = 0.02;
        mode.loop(); // Establish the false edge baseline.
        events.clear();

        pressed[0] = true;
        mode.runtimeSec = 0.04;
        mode.loop();

        assertEquals(
                Arrays.asList(
                        "factory:1",
                        "task1.start",
                        "task1.update",
                        "output.update",
                        "realized:1",
                        "telemetry.update"
                ),
                events
        );
        assertEquals(1, created.size());
        assertEquals(created.get(0).updateClock.cycle(), output.updateClock.cycle());

        pressed[0] = false;
        mode.runtimeSec = 0.06;
        mode.loop();
        events.clear();
        pressed[0] = true;
        mode.runtimeSec = 0.08;
        mode.loop();

        assertEquals(2, created.size());
        assertNotSame(created.get(0), created.get(1));
        assertEquals(created.get(1).updateClock.cycle(), output.updateClock.cycle());
        assertEquals(Integer.valueOf(2), realizedRequests.get(realizedRequests.size() - 1));
        assertEquals(
                Long.valueOf(created.get(1).updateClock.cycle()),
                realizedCycles.get(realizedCycles.size() - 1)
        );

        mode.stop();
    }

    @Test
    public void driveHeartbeatSamplesClampsWritesAndStopsExactlyOnce() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingDriveSource source = new RecordingDriveSource(events);
        RecordingDriveSink sink = new RecordingDriveSink(events);
        DriveCommandSink[] returned = new DriveCommandSink[1];
        source.signal = new DriveSignal(2.0, -3.0, 0.25);

        mode.configureAction = program -> returned[0] = program.drive(source, sink);

        mode.init();
        assertEquals(0, sink.updateCalls);
        assertEquals(0, source.sampleCalls);
        assertEquals(0, sink.driveCalls);
        assertSame(sink, returned[0]);

        events.clear();
        mode.start();
        assertEquals(
                Arrays.asList("drive.update", "drive.sample", "drive.write"),
                events
        );
        assertDriveSignal(1.0, -1.0, 0.25, sink.signals.get(0));

        events.clear();
        mode.runtimeSec = 0.02;
        mode.loop();
        assertEquals(
                Arrays.asList(
                        "drive.update",
                        "drive.sample",
                        "drive.write",
                        "telemetry.update"
                ),
                events
        );
        assertEquals(2, sink.updateCalls);
        assertEquals(2, source.sampleCalls);
        assertEquals(2, sink.driveCalls);

        mode.stop();
        mode.stop();
        assertEquals(1, sink.stopCalls);
    }

    @Test
    public void reentrantStopDuringDriveHeartbeatOrSourcePreventsEveryLaterWrite() {
        for (boolean duringStart : new boolean[]{true, false}) {
            for (boolean stopFromHeartbeat : new boolean[]{true, false}) {
                List<String> events = new ArrayList<String>();
                RecordingTelemetry telemetry = new RecordingTelemetry(events);
                TestOpMode mode = configuredMode(telemetry);
                RecordingDriveSource source = new RecordingDriveSource(events);
                RecordingDriveSink sink = new RecordingDriveSink(events);
                source.signal = new DriveSignal(0.8, -0.6, 0.4);
                mode.configureAction = program -> program.drive(source, sink);

                mode.init();
                if (!duringStart) {
                    mode.start();
                    events.clear();
                }

                int baselineUpdates = sink.updateCalls;
                int baselineSamples = source.sampleCalls;
                int baselineWrites = sink.driveCalls;
                if (stopFromHeartbeat) {
                    sink.updateAction = mode::stop;
                } else {
                    source.sampleAction = mode::stop;
                }
                events.clear();

                if (duringStart) {
                    mode.start();
                } else {
                    mode.runtimeSec = 0.02;
                    mode.loop();
                }

                String scenario = (duringStart ? "START" : "loop")
                        + (stopFromHeartbeat ? " heartbeat" : " source");
                assertEquals(scenario, baselineUpdates + 1, sink.updateCalls);
                assertEquals(
                        scenario,
                        baselineSamples + (stopFromHeartbeat ? 0 : 1),
                        source.sampleCalls
                );
                assertEquals(scenario, baselineWrites, sink.driveCalls);
                assertEquals(scenario, 1, sink.stopCalls);
                assertTrue(scenario, !events.contains("drive.write"));
                assertTrue(scenario, !events.contains("telemetry.update"));

                mode.stop();
                assertEquals(scenario, 1, sink.stopCalls);
            }
        }
    }

    @Test
    public void nonFiniteFinalDriveComponentsFailStopBeforeTheSinkWrite() {
        DriveSignal[] invalidSignals = {
                new DriveSignal(Double.NaN, 0.0, 0.0),
                new DriveSignal(0.0, Double.POSITIVE_INFINITY, 0.0),
                new DriveSignal(0.0, 0.0, Double.NEGATIVE_INFINITY)
        };
        for (boolean duringStart : new boolean[]{true, false}) {
            for (DriveSignal invalidSignal : invalidSignals) {
                List<String> events = new ArrayList<String>();
                RecordingTelemetry telemetry = new RecordingTelemetry(events);
                TestOpMode mode = configuredMode(telemetry);
                RecordingDriveSource source = new RecordingDriveSource(events);
                RecordingDriveSink sink = new RecordingDriveSink(events);
                RuntimeException cleanupFailure = new IllegalStateException("drive cleanup");
                mode.configureAction = program -> program.drive(source, sink);

                mode.init();
                if (!duringStart) {
                    mode.start();
                }
                int baselineUpdates = sink.updateCalls;
                int baselineSamples = source.sampleCalls;
                int baselineWrites = sink.driveCalls;
                source.signal = invalidSignal;
                sink.stopFailure = cleanupFailure;
                events.clear();

                RuntimeException thrown = expectRuntimeException(() -> {
                    if (duringStart) {
                        mode.start();
                    } else {
                        mode.runtimeSec = 0.02;
                        mode.loop();
                    }
                });

                String scenario = (duringStart ? "START " : "loop ") + invalidSignal;
                assertTrue(scenario, thrown instanceof IllegalArgumentException);
                assertTrue(scenario, thrown.getMessage().contains(
                        "finite axial, lateral, and omega"));
                assertArrayEquals(
                        scenario,
                        new Throwable[]{cleanupFailure},
                        thrown.getSuppressed()
                );
                assertEquals(scenario, baselineUpdates + 1, sink.updateCalls);
                assertEquals(scenario, baselineSamples + 1, source.sampleCalls);
                assertEquals(scenario, baselineWrites, sink.driveCalls);
                assertEquals(scenario, 1, sink.stopCalls);
                assertTrue(scenario, !events.contains("drive.write"));
                assertTrue(scenario, !events.contains("telemetry.update"));

                mode.stop();
                assertEquals(scenario, 1, sink.stopCalls);
            }
        }
    }

    @Test
    public void declarationMethodsReturnExactObjectsAndStableViews() {
        RecordingTelemetry telemetry = new RecordingTelemetry(new ArrayList<String>());
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service = new RecordingService("service", new ArrayList<String>());
        RecordingOutput output = new RecordingOutput("output", new ArrayList<String>());
        RecordingDriveSource source = new RecordingDriveSource(new ArrayList<String>());
        RecordingDriveSink sink = new RecordingDriveSink(new ArrayList<String>());

        mode.configureAction = program -> {
            BindingRegistrar bindings = program.bindings();
            TaskBindings taskBindings = program.taskBindings();
            assertSame(bindings, program.bindings());
            assertSame(taskBindings, program.taskBindings());
            assertSame(service, program.service(service));
            assertSame(output, program.output(output));
            assertSame(sink, program.drive(source, sink));
        };

        mode.init();
        mode.stop();
    }

    @Test
    public void ordinaryApiHasOneFrameworkConstructionPathAndNoRawPlantRegistration()
            throws Exception {
        assertEquals(0, RobotProgram.class.getConstructors().length);

        Set<String> publicDeclarationNames = new HashSet<String>();
        int publicDeclarationCount = 0;
        for (Method method : RobotProgram.class.getDeclaredMethods()) {
            if (!Modifier.isPublic(method.getModifiers())) {
                continue;
            }
            publicDeclarationCount++;
            publicDeclarationNames.add(method.getName());
            assertTrue(!Plant.class.isAssignableFrom(method.getReturnType()));
            for (Class<?> parameterType : method.getParameterTypes()) {
                assertTrue(!Plant.class.isAssignableFrom(parameterType));
            }
        }
        assertEquals(7, publicDeclarationCount);
        assertEquals(
                new HashSet<String>(Arrays.asList(
                        "bindings",
                        "taskBindings",
                        "service",
                        "output",
                        "drive",
                        "rootTask",
                        "presenter"
                )),
                publicDeclarationNames
        );

        Method configure = FtcRobotOpMode.class.getDeclaredMethod(
                "configure",
                RobotProgram.class
        );
        assertTrue(Modifier.isProtected(configure.getModifiers()));
        assertTrue(Modifier.isAbstract(configure.getModifiers()));
        for (String callback : Arrays.asList("init", "init_loop", "start", "loop", "stop")) {
            assertTrue(Modifier.isFinal(
                    FtcRobotOpMode.class.getMethod(callback).getModifiers()));
        }
    }

    @Test
    public void nullDeclarationsFailBeforeMutatingTheGraph() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service = new RecordingService("service", events);
        RecordingOutput output = new RecordingOutput("output", events);
        RecordingDriveSource source = new RecordingDriveSource(events);
        RecordingDriveSink sink = new RecordingDriveSink(events);
        RecordingTask root = new RecordingTask("root", events);

        mode.configureAction = program -> {
            expectNullPointer(() -> program.service(null));
            expectNullPointer(() -> program.output(null));
            expectNullPointer(() -> program.drive(null, sink));
            expectNullPointer(() -> program.drive(source, null));
            expectNullPointer(() -> program.rootTask(null));
            expectNullPointer(() -> program.presenter(null));
            expectNullPointer(() -> program.bindings().whileHigh(null, () -> { }));
            expectNullPointer(() -> program.bindings().whileHigh(
                    BooleanSource.constant(true),
                    null
            ));
            expectNullPointer(() -> program.taskBindings().onRise(null, () -> root));
            expectNullPointer(() -> program.taskBindings().onRise(
                    BooleanSource.constant(true),
                    null
            ));

            assertSame(service, program.service(service));
            assertSame(output, program.output(output));
            assertSame(sink, program.drive(source, sink));
            program.rootTask(root);
            program.presenter((clock, destination) -> events.add("presenter"));
        };

        mode.init();
        mode.start();
        mode.loop();
        mode.stop();

        assertEquals(1, service.stopCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(1, sink.stopCalls);
        assertEquals(1, root.startCalls);
    }

    @Test
    public void duplicateIdentitiesSecondDriveAndSecondRootDoNotPartiallyMutate() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        MultiRoleOwner shared = new MultiRoleOwner();
        RecordingOutput acceptedOutput = new RecordingOutput("acceptedOutput", events);
        RecordingDriveSource source = new RecordingDriveSource(events);
        RecordingDriveSink acceptedDrive = new RecordingDriveSink(events);
        RecordingDriveSink rejectedDrive = new RecordingDriveSink(events);
        RecordingTask acceptedRoot = new RecordingTask("acceptedRoot", events);
        RecordingTask rejectedRoot = new RecordingTask("rejectedRoot", events);

        mode.configureAction = program -> {
            assertSame(shared, program.service(shared));
            assertDuplicate(() -> program.service(shared));
            assertDuplicate(() -> program.output(shared));
            assertDuplicate(() -> program.presenter(shared));
            assertDuplicate(() -> program.rootTask(shared));
            assertDuplicate(() -> program.drive(source, shared));

            program.output(acceptedOutput);
            assertSame(acceptedDrive, program.drive(source, acceptedDrive));
            RuntimeException secondDrive = expectRuntimeException(
                    () -> program.drive(source, rejectedDrive)
            );
            assertTrue(secondDrive.getMessage().contains("already has a drive"));

            program.rootTask(acceptedRoot);
            RuntimeException secondRoot = expectRuntimeException(
                    () -> program.rootTask(rejectedRoot)
            );
            assertTrue(secondRoot.getMessage().contains("already has a root Task"));
        };

        mode.init();
        mode.start();
        mode.loop();
        mode.stop();

        assertEquals(1, shared.startCalls);
        assertEquals(1, shared.updateCalls);
        assertEquals(1, shared.stopCalls);
        assertEquals(0, shared.presenterCalls);
        assertEquals(0, shared.driveCalls);
        assertEquals(0, shared.cancelCalls);
        assertEquals(0, rejectedDrive.updateCalls);
        assertEquals(0, rejectedDrive.driveCalls);
        assertEquals(0, rejectedDrive.stopCalls);
        assertEquals(0, rejectedRoot.startCalls);
        assertEquals(0, rejectedRoot.cancelCalls);
        assertEquals(1, acceptedRoot.startCalls);
        assertEquals(1, acceptedRoot.cancelCalls);
    }

    @Test
    public void lateDeclarationsFailWithoutJoiningTheFrozenGraph() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService accepted = new RecordingService("accepted", events);
        MultiRoleOwner rejected = new MultiRoleOwner();
        RecordingDriveSource source = new RecordingDriveSource(events);
        RecordingTask rejectedRoot = new RecordingTask("rejectedRoot", events);
        int[] rejectedBindingCalls = {0};

        mode.configureAction = program -> program.service(accepted);
        mode.init();
        RobotProgram program = mode.configuredProgram;

        assertLate(() -> program.service(rejected));
        assertLate(() -> program.output(rejected));
        assertLate(() -> program.drive(source, rejected));
        assertLate(() -> program.rootTask(rejectedRoot));
        assertLate(() -> program.presenter(rejected));
        assertLate(() -> program.bindings().whileHigh(
                BooleanSource.constant(true),
                () -> rejectedBindingCalls[0]++
        ));
        assertLate(() -> program.taskBindings().onRise(
                BooleanSource.constant(false),
                () -> rejectedRoot
        ));

        mode.start();
        mode.loop();
        mode.stop();

        assertEquals(1, accepted.startCalls);
        assertEquals(1, accepted.updateCalls);
        assertEquals(1, accepted.stopCalls);
        assertEquals(0, rejected.startCalls);
        assertEquals(0, rejected.updateCalls);
        assertEquals(0, rejected.stopCalls);
        assertEquals(0, rejected.presenterCalls);
        assertEquals(0, rejected.driveCalls);
        assertEquals(0, rejectedRoot.startCalls);
        assertEquals(0, rejectedBindingCalls[0]);
    }

    @Test
    public void configurationFailureCleansOnlyAlreadyRegisteredOwnersAndPreservesPrimary() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service = new RecordingService("service", events);
        RecordingOutput output = new RecordingOutput("output", events);
        RecordingOutput neverRegistered = new RecordingOutput("neverRegistered", events);
        RuntimeException primary = new IllegalStateException("controlled configuration failure");
        RuntimeException outputCleanup = new IllegalArgumentException("output cleanup");
        RuntimeException serviceCleanup = new IllegalArgumentException("service cleanup");
        output.stopFailure = outputCleanup;
        service.stopFailure = serviceCleanup;

        mode.configureAction = program -> {
            program.service(service);
            program.output(output);
            throw primary;
        };

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertSame(primary, thrown);
        assertArrayEquals(
                new Throwable[]{outputCleanup, serviceCleanup},
                thrown.getSuppressed()
        );
        assertEquals(
                Arrays.asList("cleanup:output", "cleanup:service"),
                cleanupEvents(events)
        );
        assertEquals(0, neverRegistered.stopCalls);
        assertTerminalCallbacksDoNothing(mode, events);
    }

    @Test
    public void everyRuntimeFailurePreservesPrimaryAndOrderedCleanupSuppression() {
        for (FailurePoint point : FailurePoint.values()) {
            FailureFixture fixture = new FailureFixture();

            RuntimeException thrown = fixture.invoke(point);

            assertSame("primary at " + point, fixture.primary, thrown);
            Throwable[] expectedSuppressed = point.startsRoot()
                    ? new Throwable[]{
                            fixture.rootCleanup,
                            fixture.output1Cleanup,
                            fixture.driveCleanup,
                            fixture.output2Cleanup,
                            fixture.service2Cleanup,
                            fixture.service1Cleanup
                    }
                    : new Throwable[]{
                            fixture.output1Cleanup,
                            fixture.driveCleanup,
                            fixture.output2Cleanup,
                            fixture.service2Cleanup,
                            fixture.service1Cleanup
                    };
            assertArrayEquals("suppression at " + point, expectedSuppressed, thrown.getSuppressed());
            List<String> expectedCleanup = point.startsRoot()
                    ? Arrays.asList(
                            "cleanup:root",
                            "cleanup:output1",
                            "cleanup:drive",
                            "cleanup:output2",
                            "cleanup:service2",
                            "cleanup:service1"
                    )
                    : Arrays.asList(
                            "cleanup:output1",
                            "cleanup:drive",
                            "cleanup:output2",
                            "cleanup:service2",
                            "cleanup:service1"
                    );
            assertEquals("cleanup order at " + point, expectedCleanup, cleanupEvents(fixture.events));
            assertTerminalCallbacksDoNothing(fixture.mode, fixture.events);
        }
    }

    @Test
    public void stopBeforeStartSkipsUnstartedRootAndStopsEveryOwnerOnce() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service1 = new RecordingService("service1", events);
        RecordingService service2 = new RecordingService("service2", events);
        RecordingOutput output1 = new RecordingOutput("output1", events);
        RecordingOutput output2 = new RecordingOutput("output2", events);
        RecordingTask root = new RecordingTask("root", events);

        mode.configureAction = program -> {
            program.service(service1);
            program.service(service2);
            program.rootTask(root);
            program.output(output1);
            program.output(output2);
        };

        mode.init();
        events.clear();
        mode.stop();
        mode.stop();

        assertEquals(
                Arrays.asList(
                        "cleanup:output1",
                        "cleanup:output2",
                        "cleanup:service2",
                        "cleanup:service1"
                ),
                events
        );
        assertEquals(0, root.startCalls);
        assertEquals(0, root.cancelCalls);
        assertTerminalCallbacksDoNothing(mode, events);
    }

    @Test
    public void explicitStopDetachesBeforeReentrantCleanupAndRunsExactlyOnce() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service1 = new RecordingService("service1", events);
        RecordingService service2 = new RecordingService("service2", events);
        RecordingOutput output1 = new RecordingOutput("output1", events);
        RecordingOutput output2 = new RecordingOutput("output2", events);
        RecordingTask root = new RecordingTask("root", events);

        mode.configureAction = program -> {
            program.service(service1);
            program.service(service2);
            program.rootTask(root);
            program.output(output1);
            program.output(output2);
        };
        mode.init();
        mode.start();
        output1.stopAction = mode::stop;
        events.clear();

        mode.stop();
        mode.stop();

        assertEquals(
                Arrays.asList(
                        "cleanup:root",
                        "cleanup:output1",
                        "cleanup:output2",
                        "cleanup:service2",
                        "cleanup:service1"
                ),
                events
        );
        assertEquals(1, root.cancelCalls);
        assertEquals(1, output1.stopCalls);
        assertEquals(1, output2.stopCalls);
        assertEquals(1, service1.stopCalls);
        assertEquals(1, service2.stopCalls);
        assertTerminalCallbacksDoNothing(mode, events);
    }

    @Test
    public void stopDuringBindingSourceOrCallbackAbortsTraversalAndFinishesCleanup() {
        for (boolean stopFromSource : new boolean[]{true, false}) {
            List<String> events = new ArrayList<String>();
            RecordingTelemetry telemetry = new RecordingTelemetry(events);
            TestOpMode mode = configuredMode(telemetry);
            RecordingService service = new RecordingService("service", events);
            RecordingTask root = new RecordingTask("root", events);
            RecordingOutput output = new RecordingOutput("output", events);
            int[] presenterCalls = {0};
            boolean[] bindingUnwound = {false};
            boolean[] cleanupObservedUnwoundBinding = {false};

            mode.configureAction = program -> {
                program.service(service);
                BooleanSource firstSignal = stopFromSource
                        ? clock -> {
                            events.add("binding.source");
                            try {
                                mode.stop();
                            } finally {
                                bindingUnwound[0] = true;
                                events.add("binding.unwound");
                            }
                            events.add("binding.source.after");
                            return true;
                        }
                        : BooleanSource.constant(true);
                program.bindings().whileHigh(firstSignal, () -> {
                    events.add("binding.action");
                    try {
                        mode.stop();
                    } finally {
                        bindingUnwound[0] = true;
                        events.add("binding.unwound");
                    }
                    events.add("binding.action.after");
                });
                program.bindings().whileHigh(
                        BooleanSource.constant(true),
                        () -> events.add("binding.later")
                );
                program.rootTask(root);
                program.output(output);
                program.presenter((clock, destination) -> presenterCalls[0]++);
            };

            mode.init();
            mode.start();
            int rootUpdatesBeforeLoop = root.updateCalls;
            int outputUpdatesBeforeLoop = output.updateCalls;
            int presenterCallsBeforeLoop = presenterCalls[0];
            int commitsBeforeLoop = telemetry.updateCalls;
            output.stopAction = () ->
                    cleanupObservedUnwoundBinding[0] = bindingUnwound[0];
            events.clear();

            mode.runtimeSec = 0.02;
            mode.loop();

            String scenario = stopFromSource ? "binding source" : "binding callback";
            assertEquals(scenario, rootUpdatesBeforeLoop, root.updateCalls);
            assertEquals(scenario, outputUpdatesBeforeLoop, output.updateCalls);
            assertEquals(scenario, presenterCallsBeforeLoop, presenterCalls[0]);
            assertEquals(scenario, commitsBeforeLoop, telemetry.updateCalls);
            assertEquals(scenario, 1, root.cancelCalls);
            assertEquals(scenario, 1, output.stopCalls);
            assertEquals(scenario, 1, service.stopCalls);
            assertTrue(scenario, cleanupObservedUnwoundBinding[0]);
            assertTrue(scenario, !events.contains("binding.source.after"));
            assertTrue(scenario, !events.contains("binding.action.after"));
            assertTrue(scenario, !events.contains("binding.later"));
            assertEquals(scenario, !stopFromSource, events.contains("binding.action"));

            int eventCount = events.size();
            mode.loop();
            mode.stop();
            assertEquals(scenario, eventCount, events.size());
        }
    }

    @Test
    public void stopDuringBindingPreservesCleanupFailureAndSuppressionOrder() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service = new RecordingService("service", events);
        RecordingTask root = new RecordingTask("root", events);
        RecordingOutput output = new RecordingOutput("output", events);
        RuntimeException rootCleanup = new IllegalStateException("root cleanup");
        RuntimeException outputCleanup = new IllegalArgumentException("output cleanup");
        RuntimeException serviceCleanup = new IllegalArgumentException("service cleanup");
        root.cancelFailure = rootCleanup;
        output.stopFailure = outputCleanup;
        service.stopFailure = serviceCleanup;

        mode.configureAction = program -> {
            program.service(service);
            program.bindings().whileHigh(BooleanSource.constant(true), () -> {
                events.add("binding.action");
                try {
                    mode.stop();
                } finally {
                    events.add("binding.unwound");
                }
            });
            program.rootTask(root);
            program.output(output);
        };
        mode.init();
        mode.start();
        events.clear();

        RuntimeException actual = expectRuntimeException(mode::loop);

        assertSame(rootCleanup, actual);
        assertArrayEquals(
                new Throwable[]{outputCleanup, serviceCleanup},
                actual.getSuppressed()
        );
        assertEquals(
                Arrays.asList(
                        "service.update",
                        "binding.action",
                        "binding.unwound",
                        "cleanup:root",
                        "cleanup:output",
                        "cleanup:service"
                ),
                events
        );
        assertTerminalCallbacksDoNothing(mode, events);
    }

    @Test
    public void stopDuringAnActiveOutputPreventsLaterActuationAndPresentation() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingService service = new RecordingService("service", events);
        RecordingTask root = new RecordingTask("root", events);
        RecordingOutput output1 = new RecordingOutput("output1", events);
        RecordingOutput output2 = new RecordingOutput("output2", events);
        int[] presenterCalls = {0};

        mode.configureAction = program -> {
            program.service(service);
            program.rootTask(root);
            program.output(output1);
            program.output(output2);
            program.presenter((clock, destination) -> {
                presenterCalls[0]++;
                events.add("presenter");
            });
        };
        mode.init();
        mode.start();
        int output2CallsBeforeLoop = output2.updateCalls;
        int presenterCallsBeforeLoop = presenterCalls[0];
        int commitsBeforeLoop = telemetry.updateCalls;
        output1.updateAction = mode::stop;
        events.clear();

        mode.runtimeSec = 0.02;
        mode.loop();

        assertEquals(output2CallsBeforeLoop, output2.updateCalls);
        assertEquals(presenterCallsBeforeLoop, presenterCalls[0]);
        assertEquals(commitsBeforeLoop, telemetry.updateCalls);
        assertEquals(1, root.cancelCalls);
        assertEquals(1, output1.stopCalls);
        assertEquals(1, output2.stopCalls);
        assertEquals(1, service.stopCalls);
        int eventCount = events.size();
        mode.loop();
        mode.stop();
        assertEquals(eventCount, events.size());
    }

    @Test
    public void nonFiniteRuntimeAtEachClockBoundaryIsActionableAndFailStops() {
        for (RuntimeBoundary boundary : RuntimeBoundary.values()) {
            List<String> events = new ArrayList<String>();
            RecordingTelemetry telemetry = new RecordingTelemetry(events);
            TestOpMode mode = configuredMode(telemetry);
            RecordingService service = new RecordingService("service", events);
            RecordingOutput output = new RecordingOutput("output", events);
            RecordingTask root = new RecordingTask("root", events);
            mode.configureAction = program -> {
                program.service(service);
                program.rootTask(root);
                program.output(output);
            };

            if (boundary == RuntimeBoundary.INIT) {
                mode.runtimeSec = Double.NaN;
            } else {
                mode.init();
                if (boundary == RuntimeBoundary.LOOP) {
                    mode.start();
                }
                mode.runtimeSec = Double.POSITIVE_INFINITY;
            }

            RuntimeException thrown = expectRuntimeException(() -> boundary.invoke(mode));

            assertTrue("type at " + boundary, thrown instanceof IllegalArgumentException);
            assertTrue("message at " + boundary, thrown.getMessage().contains("must be finite"));
            if (boundary == RuntimeBoundary.INIT) {
                assertEquals(0, mode.configureCalls);
            } else {
                assertEquals(1, output.stopCalls);
                assertEquals(1, service.stopCalls);
                assertEquals(boundary == RuntimeBoundary.LOOP ? 1 : 0, root.cancelCalls);
            }
            assertTerminalCallbacksDoNothing(mode, events);
        }
    }

    @Test
    public void aSecondInitIsRejectedWithoutReplacingTheOwnedProgram() {
        List<String> events = new ArrayList<String>();
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        TestOpMode mode = configuredMode(telemetry);
        RecordingOutput output = new RecordingOutput("output", events);
        mode.configureAction = program -> program.output(output);

        mode.init();
        RobotProgram retained = mode.configuredProgram;

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown instanceof IllegalStateException);
        assertTrue(thrown.getMessage().contains("only once"));
        assertEquals(1, mode.configureCalls);
        assertSame(retained, mode.configuredProgram);
        mode.stop();
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void errorsEscapeUncaughtAndRemainExplicitlyStoppable() {
        List<String> configureEvents = new ArrayList<String>();
        RecordingTelemetry configureTelemetry = new RecordingTelemetry(configureEvents);
        TestOpMode configureMode = configuredMode(configureTelemetry);
        RecordingOutput configuredOutput = new RecordingOutput("configuredOutput", configureEvents);
        AssertionError configureError = new AssertionError("fatal configuration error");
        configureMode.configureAction = program -> {
            program.output(configuredOutput);
            throw configureError;
        };

        assertSame(configureError, expectAssertionError(configureMode::init));
        assertEquals(0, configuredOutput.stopCalls);
        configureMode.stop();
        assertEquals(1, configuredOutput.stopCalls);

        List<String> loopEvents = new ArrayList<String>();
        RecordingTelemetry loopTelemetry = new RecordingTelemetry(loopEvents);
        TestOpMode loopMode = configuredMode(loopTelemetry);
        RecordingTask root = new RecordingTask("root", loopEvents);
        RecordingOutput loopOutput = new RecordingOutput("loopOutput", loopEvents);
        loopMode.configureAction = program -> {
            program.rootTask(root);
            program.output(loopOutput);
        };
        loopMode.init();
        loopMode.start();
        AssertionError loopError = new AssertionError("fatal active error");
        loopOutput.updateError = loopError;

        assertSame(loopError, expectAssertionError(loopMode::loop));
        assertEquals(0, loopOutput.stopCalls);
        assertEquals(0, root.cancelCalls);
        loopMode.stop();
        assertEquals(1, loopOutput.stopCalls);
        assertEquals(1, root.cancelCalls);
    }

    private static void assertDuplicate(Runnable declaration) {
        RuntimeException failure = expectRuntimeException(declaration);
        assertTrue(failure instanceof IllegalStateException);
        assertTrue(failure.getMessage().contains("already registered this exact object"));
    }

    private static void assertLate(Runnable declaration) {
        RuntimeException failure = expectRuntimeException(declaration);
        assertTrue(failure instanceof IllegalStateException);
        assertTrue(failure.getMessage().contains("after configure(program) returns"));
    }

    private static void assertDriveSignal(
            double axial,
            double lateral,
            double omega,
            DriveSignal actual
    ) {
        assertEquals(axial, actual.axial, 0.0);
        assertEquals(lateral, actual.lateral, 0.0);
        assertEquals(omega, actual.omega, 0.0);
    }

    private static void assertTerminalCallbacksDoNothing(TestOpMode mode, List<String> events) {
        int eventCount = events.size();
        int configureCalls = mode.configureCalls;
        mode.init();
        mode.init_loop();
        mode.start();
        mode.loop();
        mode.stop();
        assertEquals(eventCount, events.size());
        assertEquals(configureCalls, mode.configureCalls);
    }

    private static List<String> cleanupEvents(List<String> events) {
        List<String> cleanup = new ArrayList<String>();
        for (String event : events) {
            if (event.startsWith("cleanup:")) {
                cleanup.add(event);
            }
        }
        return cleanup;
    }

    private static TestOpMode configuredMode(RecordingTelemetry telemetry) {
        TestOpMode mode = new TestOpMode();
        mode.telemetry = telemetry.proxy();
        mode.hardwareMap = new HardwareMap(null, null);
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        return mode;
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

    private static NullPointerException expectNullPointer(Runnable operation) {
        RuntimeException failure = expectRuntimeException(operation);
        assertTrue(failure instanceof NullPointerException);
        return (NullPointerException) failure;
    }

    private static AssertionError expectAssertionError(Runnable operation) {
        try {
            operation.run();
            fail("Expected AssertionError");
            throw new AssertionError("unreachable");
        } catch (AssertionError expected) {
            return expected;
        }
    }

    private enum FailurePoint {
        INIT_PRESENTER(false),
        INIT_TELEMETRY(false),
        INIT_LOOP_PRESENTER(false),
        INIT_LOOP_TELEMETRY(false),
        START_SERVICE(false),
        START_ROOT_START(true),
        START_ROOT_UPDATE(true),
        START_OUTPUT(true),
        START_DRIVE_HEARTBEAT(true),
        START_DRIVE_SOURCE(true),
        START_DRIVE_WRITE(true),
        LOOP_SERVICE(true),
        LOOP_BINDING(true),
        LOOP_TASK(true),
        LOOP_OUTPUT(true),
        LOOP_DRIVE_HEARTBEAT(true),
        LOOP_DRIVE_SOURCE(true),
        LOOP_DRIVE_WRITE(true),
        LOOP_PRESENTER(true),
        LOOP_TELEMETRY(true);

        private final boolean startsRoot;

        FailurePoint(boolean startsRoot) {
            this.startsRoot = startsRoot;
        }

        boolean startsRoot() {
            return startsRoot;
        }

        boolean isInitFailure() {
            return this == INIT_PRESENTER || this == INIT_TELEMETRY;
        }

        boolean isInitLoopFailure() {
            return this == INIT_LOOP_PRESENTER || this == INIT_LOOP_TELEMETRY;
        }

        boolean isStartFailure() {
            return name().startsWith("START_");
        }
    }

    private enum RuntimeBoundary {
        INIT {
            @Override
            void invoke(TestOpMode mode) {
                mode.init();
            }
        },
        INIT_LOOP {
            @Override
            void invoke(TestOpMode mode) {
                mode.init_loop();
            }
        },
        START {
            @Override
            void invoke(TestOpMode mode) {
                mode.start();
            }
        },
        LOOP {
            @Override
            void invoke(TestOpMode mode) {
                mode.loop();
            }
        };

        abstract void invoke(TestOpMode mode);
    }

    private static final class FailureFixture {
        final List<String> events = new ArrayList<String>();
        final RecordingTelemetry telemetry = new RecordingTelemetry(events);
        final TestOpMode mode = configuredMode(telemetry);
        final RuntimeException primary = new IllegalStateException("controlled phase failure");
        final RuntimeException rootCleanup = new IllegalArgumentException("root cleanup");
        final RuntimeException output1Cleanup = new IllegalArgumentException("output1 cleanup");
        final RuntimeException driveCleanup = new IllegalArgumentException("drive cleanup");
        final RuntimeException output2Cleanup = new IllegalArgumentException("output2 cleanup");
        final RuntimeException service2Cleanup = new IllegalArgumentException("service2 cleanup");
        final RuntimeException service1Cleanup = new IllegalArgumentException("service1 cleanup");
        final RecordingService service1 = new RecordingService("service1", events);
        final RecordingService service2 = new RecordingService("service2", events);
        final RecordingTask root = new RecordingTask("root", events);
        final RecordingOutput output1 = new RecordingOutput("output1", events);
        final RecordingOutput output2 = new RecordingOutput("output2", events);
        final RecordingDriveSource driveSource = new RecordingDriveSource(events);
        final RecordingDriveSink driveSink = new RecordingDriveSink(events);
        FailurePoint activeFailure;

        FailureFixture() {
            root.cancelFailure = rootCleanup;
            output1.stopFailure = output1Cleanup;
            driveSink.stopFailure = driveCleanup;
            output2.stopFailure = output2Cleanup;
            service2.stopFailure = service2Cleanup;
            service1.stopFailure = service1Cleanup;

            service1.startAction = () -> failAt(FailurePoint.START_SERVICE);
            service1.updateAction = () -> failAt(FailurePoint.LOOP_SERVICE);
            root.startAction = () -> failAt(FailurePoint.START_ROOT_START);
            root.updateAction = () -> {
                failAt(FailurePoint.START_ROOT_UPDATE);
                failAt(FailurePoint.LOOP_TASK);
            };
            output1.updateAction = () -> {
                failAt(FailurePoint.START_OUTPUT);
                failAt(FailurePoint.LOOP_OUTPUT);
            };
            driveSink.updateAction = () -> {
                failAt(FailurePoint.START_DRIVE_HEARTBEAT);
                failAt(FailurePoint.LOOP_DRIVE_HEARTBEAT);
            };
            driveSource.sampleAction = () -> {
                failAt(FailurePoint.START_DRIVE_SOURCE);
                failAt(FailurePoint.LOOP_DRIVE_SOURCE);
            };
            driveSink.driveAction = () -> {
                failAt(FailurePoint.START_DRIVE_WRITE);
                failAt(FailurePoint.LOOP_DRIVE_WRITE);
            };
            telemetry.updateAction = () -> {
                failAt(FailurePoint.INIT_TELEMETRY);
                failAt(FailurePoint.INIT_LOOP_TELEMETRY);
                failAt(FailurePoint.LOOP_TELEMETRY);
            };

            mode.configureAction = program -> {
                program.service(service1);
                program.service(service2);
                program.bindings().whileHigh(BooleanSource.constant(true), () -> {
                    events.add("binding");
                    failAt(FailurePoint.LOOP_BINDING);
                });
                program.rootTask(root);
                program.output(output1);
                program.drive(driveSource, driveSink);
                program.output(output2);
                program.presenter((clock, destination) -> {
                    events.add("presenter");
                    failAt(FailurePoint.INIT_PRESENTER);
                    failAt(FailurePoint.INIT_LOOP_PRESENTER);
                    failAt(FailurePoint.LOOP_PRESENTER);
                });
            };
        }

        RuntimeException invoke(FailurePoint point) {
            if (point.isInitFailure()) {
                activeFailure = point;
                return expectRuntimeException(mode::init);
            }

            mode.init();
            if (point.isInitLoopFailure()) {
                activeFailure = point;
                return expectRuntimeException(mode::init_loop);
            }
            if (point.isStartFailure()) {
                activeFailure = point;
                return expectRuntimeException(mode::start);
            }

            mode.start();
            activeFailure = point;
            return expectRuntimeException(mode::loop);
        }

        private void failAt(FailurePoint point) {
            if (activeFailure == point) {
                throw primary;
            }
        }
    }

    private static final class TestOpMode extends FtcRobotOpMode {
        Consumer<RobotProgram> configureAction = program -> { };
        RobotProgram configuredProgram;
        int configureCalls;
        double runtimeSec;

        @Override
        protected void configure(RobotProgram program) {
            configureCalls++;
            configuredProgram = program;
            configureAction.accept(program);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final class RecordingService implements RobotProgram.Service {
        final String name;
        final List<String> events;
        Runnable startAction;
        Runnable updateAction;
        Runnable stopAction;
        RuntimeException startFailure;
        RuntimeException updateFailure;
        RuntimeException stopFailure;
        AssertionError updateError;
        LoopClock startClock;
        LoopClock updateClock;
        int startCalls;
        int updateCalls;
        int stopCalls;

        RecordingService(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            startCalls++;
            startClock = clock;
            events.add(name + ".start");
            run(startAction);
            throwIfPresent(startFailure);
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            updateClock = clock;
            events.add(name + ".update");
            run(updateAction);
            if (updateError != null) {
                throw updateError;
            }
            throwIfPresent(updateFailure);
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add("cleanup:" + name);
            run(stopAction);
            throwIfPresent(stopFailure);
        }
    }

    private static final class RecordingOutput implements RobotProgram.Output {
        final String name;
        final List<String> events;
        Runnable updateAction;
        Runnable stopAction;
        RuntimeException updateFailure;
        RuntimeException stopFailure;
        AssertionError updateError;
        LoopClock updateClock;
        int updateCalls;
        int stopCalls;

        RecordingOutput(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            updateClock = clock;
            events.add(name + ".update");
            run(updateAction);
            if (updateError != null) {
                throw updateError;
            }
            throwIfPresent(updateFailure);
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add("cleanup:" + name);
            run(stopAction);
            throwIfPresent(stopFailure);
        }
    }

    private static final class RecordingDriveSource implements DriveSource {
        final List<String> events;
        DriveSignal signal = DriveSignal.zero();
        Runnable sampleAction;
        RuntimeException sampleFailure;
        LoopClock sampleClock;
        int sampleCalls;

        RecordingDriveSource(List<String> events) {
            this.events = events;
        }

        @Override
        public DriveSignal get(LoopClock clock) {
            sampleCalls++;
            sampleClock = clock;
            events.add("drive.sample");
            run(sampleAction);
            throwIfPresent(sampleFailure);
            return signal;
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        final List<String> events;
        final List<DriveSignal> signals = new ArrayList<DriveSignal>();
        Runnable updateAction;
        Runnable driveAction;
        Runnable stopAction;
        RuntimeException updateFailure;
        RuntimeException driveFailure;
        RuntimeException stopFailure;
        LoopClock updateClock;
        int updateCalls;
        int driveCalls;
        int stopCalls;

        RecordingDriveSink(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            updateClock = clock;
            events.add("drive.update");
            run(updateAction);
            throwIfPresent(updateFailure);
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCalls++;
            signals.add(signal);
            events.add("drive.write");
            run(driveAction);
            throwIfPresent(driveFailure);
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add("cleanup:drive");
            run(stopAction);
            throwIfPresent(stopFailure);
        }
    }

    private static class RecordingTask implements Task {
        final String name;
        final List<String> events;
        Runnable startAction;
        Runnable updateAction;
        Runnable cancelAction;
        RuntimeException startFailure;
        RuntimeException updateFailure;
        RuntimeException cancelFailure;
        LoopClock startClock;
        LoopClock updateClock;
        boolean completeOnUpdate;
        boolean started;
        boolean complete;
        boolean cancelled;
        int startCalls;
        int updateCalls;
        int cancelCalls;

        RecordingTask(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            startCalls++;
            started = true;
            startClock = clock;
            events.add(name + ".start");
            run(startAction);
            throwIfPresent(startFailure);
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            updateClock = clock;
            events.add(name + ".update");
            run(updateAction);
            throwIfPresent(updateFailure);
            if (completeOnUpdate) {
                complete = true;
            }
        }

        @Override
        public void cancel() {
            if (!started || complete || cancelled) {
                return;
            }
            cancelled = true;
            cancelCalls++;
            events.add("cleanup:" + name);
            run(cancelAction);
            throwIfPresent(cancelFailure);
        }

        @Override
        public boolean isComplete() {
            return complete || cancelled;
        }

        @Override
        public TaskOutcome getOutcome() {
            if (cancelled) {
                return TaskOutcome.CANCELLED;
            }
            return complete ? TaskOutcome.SUCCESS : TaskOutcome.NOT_DONE;
        }
    }

    private static final class MultiRoleOwner implements
            RobotProgram.Service,
            RobotProgram.Output,
            RobotProgram.Presenter,
            DriveCommandSink,
            Task {
        int startCalls;
        int updateCalls;
        int stopCalls;
        int presenterCalls;
        int driveCalls;
        int cancelCalls;

        @Override
        public void start(LoopClock clock) {
            startCalls++;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
        }

        @Override
        public void stop() {
            stopCalls++;
        }

        @Override
        public void present(LoopClock clock, Telemetry telemetry) {
            presenterCalls++;
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCalls++;
        }

        @Override
        public void cancel() {
            cancelCalls++;
        }

        @Override
        public boolean isComplete() {
            return false;
        }

        @Override
        public TaskOutcome getOutcome() {
            return TaskOutcome.NOT_DONE;
        }
    }

    private static final class ClockSnapshot {
        final long cycle;
        final double nowSec;
        final double dtSec;

        ClockSnapshot(long cycle, double nowSec, double dtSec) {
            this.cycle = cycle;
            this.nowSec = nowSec;
            this.dtSec = dtSec;
        }

        static ClockSnapshot of(LoopClock clock) {
            return new ClockSnapshot(clock.cycle(), clock.nowSec(), clock.dtSec());
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof ClockSnapshot)) {
                return false;
            }
            ClockSnapshot that = (ClockSnapshot) other;
            return cycle == that.cycle
                    && Double.compare(nowSec, that.nowSec) == 0
                    && Double.compare(dtSec, that.dtSec) == 0;
        }

        @Override
        public int hashCode() {
            long nowBits = Double.doubleToLongBits(nowSec);
            long dtBits = Double.doubleToLongBits(dtSec);
            int result = (int) (cycle ^ (cycle >>> 32));
            result = 31 * result + (int) (nowBits ^ (nowBits >>> 32));
            result = 31 * result + (int) (dtBits ^ (dtBits >>> 32));
            return result;
        }

        @Override
        public String toString() {
            return "ClockSnapshot{" + cycle + ", " + nowSec + ", " + dtSec + '}';
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> events;
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );
        Runnable updateAction;
        int updateCalls;

        RecordingTelemetry(List<String> events) {
            this.events = events;
        }

        Telemetry proxy() {
            return proxy;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if ("update".equals(method.getName())) {
                updateCalls++;
                events.add("telemetry.update");
                run(updateAction);
            }
            return defaultValue(method.getReturnType());
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

    private static void run(Runnable action) {
        if (action != null) {
            action.run();
        }
    }

    private static void throwIfPresent(RuntimeException failure) {
        if (failure != null) {
            throw failure;
        }
    }
}
