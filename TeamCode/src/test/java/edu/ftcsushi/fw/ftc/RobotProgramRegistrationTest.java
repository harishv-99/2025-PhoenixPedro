package edu.ftcsushi.fw.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskOutcome;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Checks transfer of returned owners, without claiming discovery of hidden resource aliases. */
public final class RobotProgramRegistrationTest {
    private static final DriveSource ZERO = clock -> DriveSignal.zero();
    private static final Role[] RESOURCE_ROLES = {Role.SERVICE, Role.OUTPUT, Role.DRIVE};

    @Test
    public void identityAcceptedInAnyRoleIsNeverStoppedByARejectedOwningDeclaration() {
        for (Role acceptedRole : Role.values()) {
            RobotProgram program = program();
            Owner accepted = new Owner("accepted");
            acceptedRole.declare(program, accepted);
            for (Role attemptedRole : RESOURCE_ROLES) {
                assertThrows(RuntimeException.class, () -> attemptedRole.declare(program, accepted));
                assertEquals(0, accepted.stops);
            }
            // Even another invalid argument must not turn a known sink into a fresh owner.
            assertThrows(NullPointerException.class, () -> program.drive(null, accepted));
            assertEquals(0, accepted.stops);
            program.stop();
            program.stop();
            assertEquals(acceptedRole.ownsStop() ? 1 : 0, accepted.stops);
            assertEquals(0, accepted.cancels);
            assertEquals(0, accepted.equalsCalls);
            assertEquals(0, accepted.hashCodeCalls);
        }
    }

    @Test
    public void equalButDistinctObjectsTransferIndependentlyWithoutUsingEquality() {
        RobotProgram program = program();
        Owner first = new Owner("first");
        Owner second = new Owner("second");
        assertSame(first, program.service(first));
        assertSame(second, program.service(second));
        assertEquals(0, first.equalsCalls + second.equalsCalls);
        assertEquals(0, first.hashCodeCalls + second.hashCodeCalls);
        program.stop();
        assertEquals(1, first.stops);
        assertEquals(1, second.stops);
    }

    @Test
    public void nullSourceStopsTheFreshSinkAndItsIdentityCannotBeReusedInAnyRole() {
        RobotProgram program = program();
        Owner rejected = new Owner("rejected");
        NullPointerException failure = assertThrows(NullPointerException.class,
                () -> program.drive(null, rejected));
        assertTrue(failure.getMessage().contains("drive source"));
        assertEquals(1, rejected.stops);
        for (int repeat = 0; repeat < 2; repeat++) {
            for (Role role : Role.values()) {
                assertThrows(RuntimeException.class, () -> role.declare(program, rejected));
            }
            assertThrows(NullPointerException.class, () -> program.drive(null, rejected));
        }
        Owner fresh = new Owner("fresh");
        assertSame(fresh, program.drive(ZERO, fresh));
        program.finishConfiguration();
        program.start(0.0);
        program.loop(0.1);
        program.stop();
        assertEquals(1, rejected.stops);
        assertEquals(0, rejected.starts + rejected.updates + rejected.drives + rejected.presents);
        assertEquals(1, fresh.stops);
        assertEquals(2, fresh.drives);
    }

    @Test
    public void lateFreshOwnersAreStoppedOnceInEveryNonConfiguringState() {
        for (Phase phase : Phase.values()) {
            for (Role role : RESOURCE_ROLES) {
                RobotProgram program = program();
                Owner accepted = new Owner("accepted");
                Owner rejected = new Owner("rejected");
                program.output(accepted);
                Runnable reject = () -> {
                    RuntimeException failure = assertThrows(RuntimeException.class,
                            () -> role.declare(program, rejected));
                    assertTrue(failure.getMessage().contains("after configure(program) returns"));
                    assertEquals(1, rejected.stops);
                    for (Role repeatedRole : RESOURCE_ROLES) {
                        assertThrows(RuntimeException.class,
                                () -> repeatedRole.declare(program, rejected));
                    }
                    assertEquals(1, rejected.stops);
                };
                if (phase == Phase.BLOCKED) {
                    Owner prestart = new Owner("prestart");
                    prestart.disposition = RobotProgram.StartDisposition.BLOCKED;
                    program.prestart(prestart);
                }
                if (phase == Phase.STARTING) {
                    Owner startObserver = new Owner("startObserver");
                    startObserver.onStart = reject;
                    program.service(startObserver);
                }
                program.finishConfiguration();
                if (phase == Phase.STARTING || phase == Phase.ACTIVE || phase == Phase.BLOCKED) {
                    program.start(0.0);
                }
                if (phase == Phase.TERMINAL) {
                    program.stop();
                }
                if (phase != Phase.STARTING) {
                    reject.run();
                }
                program.stop();
                program.stop();
                assertEquals(1, accepted.stops);
                assertEquals(1, rejected.stops);
                assertEquals(0, rejected.starts + rejected.updates + rejected.drives);
            }
        }
    }

    @Test
    public void knownIdentitiesRemainProtectedAfterTheProgramHasStopped() {
        for (Role acceptedRole : Role.values()) {
            RobotProgram program = program();
            Owner known = new Owner("known");
            acceptedRole.declare(program, known);
            program.stop();
            int stopsBefore = known.stops;
            for (Role role : RESOURCE_ROLES) {
                assertThrows(RuntimeException.class, () -> role.declare(program, known));
            }
            assertEquals(stopsBefore, known.stops);
        }
    }

    @Test
    public void rejectedDriveNeverSamplesResetsOrStopsItsBorrowedSource() {
        RobotProgram program = program();
        BorrowedSource source = new BorrowedSource();
        Owner accepted = new Owner("accepted");
        Owner rejected = new Owner("rejected");
        program.drive(source, accepted);
        assertThrows(RuntimeException.class, () -> program.drive(source, rejected));
        assertEquals(1, rejected.stops);
        assertEquals(0, source.samples + source.resets + source.stops);
        program.stop();
        assertEquals(1, accepted.stops);
        assertEquals(0, source.samples + source.resets + source.stops);

        // Being borrowed as a source also does not reserve that object's identity as an owner.
        RobotProgram secondProgram = program();
        BorrowedSource explicitlyOwnedSource = new BorrowedSource();
        secondProgram.drive(explicitlyOwnedSource, new Owner("sink"));
        assertSame(explicitlyOwnedSource, secondProgram.service(explicitlyOwnedSource));
        secondProgram.stop();
        assertEquals(1, explicitlyOwnedSource.stops);
    }

    @Test
    public void rejectionStopsOnlyTheOfferedOwnerBeforeNormalAcceptedCleanupOrder() {
        List<String> events = new ArrayList<>();
        RobotProgram program = program();
        Owner service1 = new Owner("service1", events);
        Owner service2 = new Owner("service2", events);
        Owner output1 = new Owner("output1", events);
        Owner output2 = new Owner("output2", events);
        Owner drive = new Owner("drive", events);
        Owner rejected = new Owner("rejected", events);
        program.service(service1);
        program.output(output1);
        program.drive(ZERO, drive);
        program.output(output2);
        program.service(service2);
        assertThrows(RuntimeException.class, () -> program.drive(ZERO, rejected));
        assertEquals(Arrays.asList("rejected.stop"), events);
        program.stop();
        assertEquals(Arrays.asList("rejected.stop", "output1.stop", "drive.stop",
                "output2.stop", "service2.stop", "service1.stop"), events);
    }

    @Test
    public void rejectionCleanupBlocksEveryDeclarationAndBindingWithoutInvokingTheirCallbacks() {
        RobotProgram program = program();
        Owner rejected = new Owner("rejected");
        Owner prestart = new Owner("prestart");
        Owner presenter = new Owner("presenter");
        Owner root = new Owner("root");
        List<Owner> nestedOwners = new ArrayList<>();
        int[] effects = {0};
        Runnable effect = () -> effects[0]++;
        BooleanSource signal = clock -> { effects[0]++; return true; };
        ScalarSource scalar = clock -> { effects[0]++; return 1.0; };
        rejected.onStop = () -> {
            CallbackBindings callbacks = program.callbackBindings();
            TaskBindings tasks = program.taskBindings();
            assertSame(callbacks, program.callbackBindings());
            assertSame(tasks, program.taskBindings());
            assertGuarded(() -> program.prestart(prestart));
            assertGuarded(() -> program.presenter(presenter));
            assertGuarded(() -> program.rootTask(root));
            assertGuarded(() -> program.stopHandoff(() -> { effects[0]++; return "value"; },
                    value -> effects[0]++, effect));
            assertGuarded(() -> callbacks.onRise(signal, effect));
            assertGuarded(() -> callbacks.onFall(signal, effect));
            assertGuarded(() -> callbacks.mirrorOnChange(signal, value -> effects[0]++));
            assertGuarded(() -> callbacks.whileHigh(signal, effect));
            assertGuarded(() -> callbacks.whileLow(signal, effect));
            assertGuarded(() -> callbacks.toggleOnRise(signal, effect, effect));
            assertGuarded(() -> callbacks.toggleOnRise(signal, value -> effects[0]++));
            assertGuarded(() -> callbacks.nudgeOnRise(signal, signal, 1.0, value -> effects[0]++));
            assertGuarded(() -> callbacks.copyEachCycle(scalar, value -> effects[0]++));
            assertGuarded(() -> tasks.onRise(signal, () -> { effects[0]++; return root; }));
            assertGuarded(() -> tasks.onFall(signal, () -> { effects[0]++; return root; }));
            assertGuarded(() -> tasks.mirrorOnChange(signal, value -> { effects[0]++; return root; }));
            assertGuarded(() -> tasks.toggleOnRise(signal,
                    () -> { effects[0]++; return root; }, () -> { effects[0]++; return root; }));
            assertGuarded(() -> tasks.nudgeOnRise(signal, signal, 1.0,
                    value -> { effects[0]++; return root; }));
            for (Role role : RESOURCE_ROLES) {
                Owner nested = new Owner("nested" + role);
                nestedOwners.add(nested);
                assertGuarded(() -> role.declare(program, nested));
                assertEquals(1, nested.stops);
            }
            assertEquals(0, effects[0]);
        };
        assertThrows(NullPointerException.class, () -> program.drive(null, rejected));

        // Non-owning rejected declarations reserve no identities; the guard is now restored.
        program.prestart(prestart);
        program.presenter(presenter);
        program.rootTask(root);
        Owner fresh = program.output(new Owner("fresh"));
        int[] normalCallbackCalls = {0};
        program.callbackBindings().whileHigh(BooleanSource.constant(true), () -> normalCallbackCalls[0]++);
        program.finishConfiguration();
        program.start(0.0);
        program.loop(0.1);
        program.stop();
        assertEquals(0, effects[0]);
        assertEquals(1, normalCallbackCalls[0]);
        assertEquals(1, fresh.stops);
        assertEquals(0, prestart.stops + presenter.stops + root.stops);
        for (Owner nested : nestedOwners) {
            assertEquals(1, nested.stops);
            assertEquals(0, nested.starts + nested.updates + nested.drives);
        }
    }

    @Test
    public void nestedCleanupFailureRetainsBothGuardsAndNeverReplaysAnOwnerStop() {
        RobotProgram program = program();
        Owner outer = new Owner("outer");
        Owner inner = new Owner("inner");
        RuntimeException innerCleanup = new IllegalArgumentException("inner cleanup");
        RuntimeException outerCleanup = new IllegalArgumentException("outer cleanup");
        RuntimeException[] innerRejection = {null};
        inner.onStop = () -> {
            for (Role role : RESOURCE_ROLES) {
                assertGuarded(() -> role.declare(program, outer));
                assertGuarded(() -> role.declare(program, inner));
            }
            assertEquals(1, outer.stops);
            assertEquals(1, inner.stops);
            throw innerCleanup;
        };
        outer.onStop = () -> {
            innerRejection[0] = assertGuarded(() -> program.service(inner));
            assertArrayEquals(new Throwable[]{innerCleanup}, innerRejection[0].getSuppressed());
            // An inner finally must restore the outer guard, not clear it.
            assertGuarded(() -> program.presenter(new Owner("notRegistered")));
            assertGuarded(() -> program.output(outer));
            assertGuarded(() -> program.drive(ZERO, inner));
            throw outerCleanup;
        };
        NullPointerException primary = assertThrows(NullPointerException.class,
                () -> program.drive(null, outer));
        assertArrayEquals(new Throwable[]{outerCleanup}, primary.getSuppressed());
        assertArrayEquals(new Throwable[]{innerCleanup}, innerRejection[0].getSuppressed());
        for (Role role : RESOURCE_ROLES) {
            assertThrows(RuntimeException.class, () -> role.declare(program, outer));
            assertThrows(RuntimeException.class, () -> role.declare(program, inner));
        }
        Owner fresh = program.service(new Owner("fresh"));
        program.stop();
        assertEquals(1, outer.stops);
        assertEquals(1, inner.stops);
        assertEquals(1, fresh.stops);
    }

    @Test
    public void rejectionCleanupCanStopTheProgramWithoutEnteringItsOwnCleanupList() {
        RobotProgram program = program();
        List<String> events = new ArrayList<>();
        Owner service = new Owner("service", events);
        Owner output = new Owner("output", events);
        Owner rejected = new Owner("rejected", events);
        Owner nestedAfterStop = new Owner("nestedAfterStop", events);
        program.service(service);
        program.output(output);
        rejected.onStop = () -> {
            program.stop();
            assertTrue(program.isTerminal());
            assertThrows(RuntimeException.class, () -> program.service(nestedAfterStop));
            assertThrows(RuntimeException.class, () -> program.output(rejected));
            assertThrows(RuntimeException.class, () -> program.service(service));
        };
        assertThrows(NullPointerException.class, () -> program.drive(null, rejected));
        program.stop();
        assertEquals(Arrays.asList("rejected.stop", "output.stop", "service.stop",
                "nestedAfterStop.stop"), events);
        assertEquals(1, rejected.stops);
        assertEquals(1, nestedAfterStop.stops);
    }

    @Test
    public void hostRetainsTheOriginalRejectionAndOrderedCleanupFailures() {
        List<String> events = new ArrayList<>();
        Owner service = new Owner("service", events);
        Owner output = new Owner("output", events);
        Owner rejected = new Owner("rejected", events);
        RuntimeException rejectedCleanup = new IllegalArgumentException("rejected cleanup");
        RuntimeException outputCleanup = new IllegalArgumentException("output cleanup");
        RuntimeException serviceCleanup = new IllegalArgumentException("service cleanup");
        rejected.onStop = () -> { throw rejectedCleanup; };
        output.onStop = () -> { throw outputCleanup; };
        service.onStop = () -> { throw serviceCleanup; };
        RuntimeException[] observedPrimary = {null};
        Host host = host(program -> {
            program.service(service);
            program.output(output);
            try {
                program.drive(null, rejected);
            } catch (RuntimeException failure) {
                observedPrimary[0] = failure;
                throw failure;
            }
        });
        RuntimeException thrown = assertThrows(RuntimeException.class, host::init);
        assertSame(observedPrimary[0], thrown);
        assertTrue(thrown instanceof NullPointerException);
        assertArrayEquals(new Throwable[]{rejectedCleanup, outputCleanup, serviceCleanup},
                thrown.getSuppressed());
        assertEquals(Arrays.asList("rejected.stop", "output.stop", "service.stop"), events);
        host.stop();
        host.start();
        host.loop();
        assertEquals(1, rejected.stops);
        assertEquals(1, output.stops);
        assertEquals(1, service.stops);
    }

    @Test
    public void activeRejectedOwnerReentrantHostStopInvalidatesHandoffWithoutCapture() {
        for (Role role : RESOURCE_ROLES) {
            List<String> events = new ArrayList<>();
            Owner accepted = new Owner("accepted", events);
            Owner rejected = new Owner("rejected", events);
            RobotProgram[] retained = {null};
            int[] captures = {0};
            int[] publications = {0};
            int[] invalidations = {0};
            Host host = host(program -> {
                retained[0] = program;
                program.output(accepted);
                program.stopHandoff(() -> { captures[0]++; return "value"; },
                        value -> publications[0]++, () -> {
                            invalidations[0]++;
                            events.add("handoff.invalidate");
                        });
            });
            host.init();
            host.start();
            events.clear();
            rejected.onStop = host::stop;

            RuntimeException failure = assertThrows(RuntimeException.class,
                    () -> role.declare(retained[0], rejected));

            assertTrue(failure.getMessage().contains("after configure(program) returns"));
            assertEquals(0, failure.getSuppressed().length);
            assertTrue(retained[0].isTerminal());
            assertEquals(0, captures[0]);
            assertEquals(0, publications[0]);
            assertEquals(2, invalidations[0]);
            assertEquals(Arrays.asList("rejected.stop", "accepted.stop", "handoff.invalidate"),
                    events);
            host.stop();
            host.start();
            host.loop();
            assertEquals(1, accepted.stops);
            assertEquals(1, rejected.stops);
            assertEquals(1, accepted.updates);
            assertEquals(0, rejected.starts + rejected.updates + rejected.drives);
            assertEquals(2, invalidations[0]);
        }
    }

    @Test
    public void bindingRejectionStopDefersCleanupAndNeverPublishesEvenWhenFailureIsCaught() {
        for (boolean catchRegistrationFailure : new boolean[]{false, true}) {
            List<String> events = new ArrayList<>();
            Owner service = new Owner("service", events);
            Owner output = new Owner("output", events);
            Owner rejected = new Owner("rejected", events);
            Owner root = new Owner("root", events);
            RuntimeException outputFailure = new IllegalArgumentException("output cleanup");
            RuntimeException serviceFailure = new IllegalArgumentException("service cleanup");
            RuntimeException invalidationFailure = new IllegalArgumentException("invalidation");
            RuntimeException[] observedPrimary = {null};
            boolean[] bindingUnwound = {false};
            int[] laterBindingCalls = {0};
            int[] captures = {0};
            int[] publications = {0};
            int[] invalidations = {0};
            Host host = host(program -> {
                program.service(service);
                program.output(output);
                program.rootTask(root);
                program.callbackBindings().whileHigh(BooleanSource.constant(true), () -> {
                    events.add("binding.action");
                    try {
                        program.output(rejected);
                    } catch (RuntimeException failure) {
                        observedPrimary[0] = failure;
                        if (!catchRegistrationFailure) {
                            throw failure;
                        }
                    } finally {
                        bindingUnwound[0] = true;
                        events.add("binding.unwound");
                    }
                });
                program.callbackBindings().whileHigh(BooleanSource.constant(true),
                        () -> laterBindingCalls[0]++);
                program.stopHandoff(() -> { captures[0]++; return "value"; },
                        value -> publications[0]++, () -> {
                            invalidations[0]++;
                            events.add("handoff.invalidate");
                            if (!catchRegistrationFailure && invalidations[0] > 1) {
                                throw invalidationFailure;
                            }
                        });
            });
            output.onStop = () -> {
                assertTrue(bindingUnwound[0]);
                if (!catchRegistrationFailure) {
                    throw outputFailure;
                }
            };
            service.onStop = () -> {
                assertTrue(bindingUnwound[0]);
                if (!catchRegistrationFailure) {
                    throw serviceFailure;
                }
            };
            rejected.onStop = host::stop;
            host.init();
            host.start();
            int rootUpdatesBeforeLoop = root.updates;
            events.clear();

            if (catchRegistrationFailure) {
                host.loop();
            } else {
                RuntimeException actual = assertThrows(RuntimeException.class, host::loop);
                assertSame(observedPrimary[0], actual);
            }

            assertTrue(observedPrimary[0].getMessage().contains("after configure(program) returns"));
            Throwable[] suppressed = observedPrimary[0].getSuppressed();
            assertEquals(catchRegistrationFailure ? 1 : 4, suppressed.length);
            // The private cooperative STOP token does not replace the registration failure.
            assertEquals("BindingTraversalStopped", suppressed[0].getClass().getSimpleName());
            if (!catchRegistrationFailure) {
                assertArrayEquals(new Throwable[]{outputFailure, serviceFailure, invalidationFailure},
                        Arrays.copyOfRange(suppressed, 1, suppressed.length));
            }
            assertEquals(Arrays.asList("binding.action", "rejected.stop", "binding.unwound",
                    "output.stop", "service.stop", "handoff.invalidate"), events);
            assertEquals(0, laterBindingCalls[0]);
            assertEquals(rootUpdatesBeforeLoop, root.updates);
            assertEquals(1, root.cancels);
            assertEquals(0, captures[0]);
            assertEquals(0, publications[0]);
            assertEquals(2, invalidations[0]);
            host.stop();
            host.loop();
            assertEquals(1, output.stops);
            assertEquals(1, service.stops);
            assertEquals(1, rejected.stops);
            assertEquals(1, output.updates);
            assertEquals(2, invalidations[0]);
        }
    }

    @Test
    public void cleanupErrorEscapesUncaughtButRestoresTheGuardAndRetainsRejectedIdentity() {
        RobotProgram program = program();
        Owner accepted = program.output(new Owner("accepted"));
        Owner rejected = new Owner("rejected");
        AssertionError fatal = new AssertionError("fatal cleanup");
        rejected.onStop = () -> { throw fatal; };
        assertSame(fatal, assertThrows(AssertionError.class, () -> program.drive(null, rejected)));
        assertFalse(program.isTerminal());
        assertEquals(0, accepted.stops);
        Owner fresh = program.service(new Owner("fresh"));
        assertThrows(RuntimeException.class, () -> program.output(rejected));
        program.stop();
        assertEquals(1, accepted.stops);
        assertEquals(1, fresh.stops);
        assertEquals(1, rejected.stops);
    }

    @Test
    public void returnedOwnerAfterConstructorReentrantStopIsCleanedAtRegistrationEntry() {
        List<String> events = new ArrayList<>();
        Owner accepted = new Owner("accepted", events);
        Owner[] created = {null};
        Host host = new Host();
        host.telemetry = telemetry();
        host.configuration = program -> {
            program.output(accepted);
            program.output(created[0] = new ConstructingOwner("late", events, host::stop));
        };
        assertThrows(RuntimeException.class, host::init);
        assertEquals(Arrays.asList("accepted.stop", "late.stop"), events);
        assertEquals(1, created[0].stops);
        host.stop();
        host.start();
        host.loop();
        assertEquals(1, accepted.stops);
        assertEquals(1, created[0].stops);
    }

    @Test
    public void constructorFailureBeforeRegistrationRemainsWithTheConstructorAndHost() {
        List<String> events = new ArrayList<>();
        Owner accepted = new Owner("accepted", events);
        RuntimeException constructorFailure = new IllegalArgumentException("constructor failure");
        int[] constructorCleanup = {0};
        Host host = host(program -> {
            program.output(accepted);
            program.output(new ConstructingOwner("neverReturned", events, () -> {
                // A constructor must handle its own unreturned partial resources.
                constructorCleanup[0]++;
                throw constructorFailure;
            }));
        });
        assertSame(constructorFailure, assertThrows(RuntimeException.class, host::init));
        assertEquals(1, constructorCleanup[0]);
        assertEquals(Arrays.asList("accepted.stop"), events);
        host.stop();
        assertEquals(1, accepted.stops);
    }

    @Test
    public void argumentFailureBeforeMethodEntryCannotTransferAnExistingLocalSink() {
        Owner accepted = new Owner("accepted");
        Owner neverOffered = new Owner("neverOffered");
        RuntimeException argumentFailure = new IllegalArgumentException("source expression failed");
        Host host = host(program -> {
            program.output(accepted);
            program.drive(failingSourceExpression(argumentFailure), neverOffered);
        });
        assertSame(argumentFailure, assertThrows(RuntimeException.class, host::init));
        assertEquals(1, accepted.stops);
        assertEquals(0, neverOffered.stops);
    }

    private static RuntimeException assertGuarded(Runnable declaration) {
        RuntimeException failure = assertThrows(RuntimeException.class, declaration::run);
        assertTrue(failure.getMessage().contains("during rejected-owner cleanup"));
        return failure;
    }

    private static DriveSource failingSourceExpression(RuntimeException failure) {
        throw failure;
    }

    private static RobotProgram program() {
        RobotProgram program = new RobotProgram(telemetry());
        program.beginInit(0.0);
        return program;
    }

    private static Host host(Consumer<RobotProgram> configuration) {
        Host host = new Host();
        host.configuration = configuration;
        host.telemetry = telemetry();
        return host;
    }

    private static Telemetry telemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                RobotProgramRegistrationTest.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, arguments) -> method.getReturnType() == boolean.class ? true : null);
    }

    private enum Phase { READY, STARTING, ACTIVE, BLOCKED, TERMINAL }

    private enum Role {
        SERVICE, OUTPUT, DRIVE, PRESTART, PRESENTER, ROOT;

        void declare(RobotProgram program, Owner owner) {
            switch (this) {
                case SERVICE: program.service(owner); break;
                case OUTPUT: program.output(owner); break;
                case DRIVE: program.drive(ZERO, owner); break;
                case PRESTART: program.prestart(owner); break;
                case PRESENTER: program.presenter(owner); break;
                case ROOT: program.rootTask(owner); break;
                default: throw new AssertionError(this);
            }
        }

        boolean ownsStop() {
            return this == SERVICE || this == OUTPUT || this == DRIVE;
        }
    }

    /** Identity-only adversary: equality deliberately equates distinct owners. */
    private static class Owner implements RobotProgram.Service, RobotProgram.Output,
            DriveCommandSink, RobotProgram.Prestart, RobotProgram.Presenter, Task {
        final String name;
        final List<String> events;
        Runnable onStart = () -> { };
        Runnable onStop = () -> { };
        RobotProgram.StartDisposition disposition = RobotProgram.StartDisposition.READY;
        int starts;
        int updates;
        int stops;
        int drives;
        int presents;
        int cancels;
        int equalsCalls;
        int hashCodeCalls;
        boolean complete;

        Owner(String name) {
            this(name, new ArrayList<>());
        }

        Owner(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override public void start(LoopClock clock) { starts++; onStart.run(); }
        @Override public void update(LoopClock clock) { updates++; }
        @Override public void drive(DriveSignal signal) { drives++; }
        @Override public void present(LoopClock clock, Telemetry telemetry) { presents++; }
        @Override public RobotProgram.StartDisposition freezeForStart() { return disposition; }
        @Override public boolean isComplete() { return complete; }
        @Override public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
        @Override public void cancel() {
            if (starts > 0 && !complete) { complete = true; cancels++; }
        }
        @Override public void stop() {
            stops++;
            events.add(name + ".stop");
            onStop.run();
        }
        @Override public boolean equals(Object other) { equalsCalls++; return other instanceof Owner; }
        @Override public int hashCode() { hashCodeCalls++; return 1; }
    }

    private static final class ConstructingOwner extends Owner {
        ConstructingOwner(String name, List<String> events, Runnable duringConstruction) {
            super(name, events);
            duringConstruction.run();
        }
    }

    private static final class BorrowedSource implements DriveSource, RobotProgram.Service {
        int samples;
        int resets;
        int stops;
        @Override public DriveSignal get(LoopClock clock) { samples++; return DriveSignal.zero(); }
        @Override public void reset() { resets++; }
        @Override public void update(LoopClock clock) { }
        @Override public void stop() { stops++; }
    }

    private static final class Host extends FtcRobotOpMode {
        Consumer<RobotProgram> configuration;
        @Override protected void configure(RobotProgram program) { configuration.accept(program); }
        @Override public double getRuntime() { return 0.0; }
    }
}
