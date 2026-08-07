package edu.ftcphoenix.fw.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.input.binding.BindingRegistrar;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskBindings;
import edu.ftcphoenix.fw.task.TaskRunner;

/**
 * Declares one FTC robot program whose reusable lifecycle is owned by
 * {@link FtcRobotOpMode}.
 *
 * <p>Ordinary robot code receives this object only in
 * {@link FtcRobotOpMode#configure(RobotProgram)}. It constructs robot-specific owners and declares
 * their semantic roles once. The framework then owns the one clock, fixed phase order, Task
 * runner, telemetry commit, and fail-stop cleanup. The declaration graph freezes when
 * {@code configure(...)} returns.</p>
 *
 * <p>The active order is:</p>
 *
 * <pre>
 * LoopClock -&gt; Services -&gt; Bindings -&gt; Tasks -&gt; Outputs/Drive -&gt; Presenters -&gt; commit
 * </pre>
 *
 * <p>There is deliberately no raw Plant registration. A mechanism or subsystem privately owns
 * its final Plant graph and implements {@link Output}, preserving one truthful Plant heartbeat and
 * stop owner.</p>
 */
public final class RobotProgram {

    /**
     * Upstream loop owner such as localization, sensing, or a vendor integration heartbeat.
     *
     * <p>A service must tolerate {@link #stop()} before {@link #start(LoopClock)} and must make
     * repeated internal cleanup harmless. The program invokes services in declaration order and
     * stops them in reverse declaration order.</p>
     */
    public interface Service {

        /**
         * Start this service at the exact FTC START clock boundary.
         *
         * <p>The default is a no-op. Use this hook only for genuine start-boundary work such as
         * applying a declared starting pose. The framework does not call {@link #update(LoopClock)}
         * implicitly from this default.</p>
         *
         * @param clock shared program clock, freshly reset at FTC START
         */
        default void start(LoopClock clock) {
            // Optional exact-start hook.
        }

        /**
         * Advance this service once in the upstream service phase.
         *
         * @param clock shared program clock for the current active cycle
         */
        void update(LoopClock clock);

        /** Stop resources owned by this service. */
        void stop();
    }

    /**
     * Downstream realization owner such as a Plant-backed mechanism.
     *
     * <p>Outputs run in declaration order after bindings and Tasks, so decisions made earlier in
     * the cycle are visible to their final actuator owner. They stop in that same declaration
     * order. An owner of several Plants keeps their internal update and stop order private.</p>
     *
     * <p>An output must tolerate {@link Output#stop()} before its first update when a later
     * configuration step fails, and should make its own repeated cleanup harmless.</p>
     */
    public interface Output {

        /**
         * Realize the current robot request once for this cycle.
         *
         * @param clock shared program clock for the current active cycle
         */
        void update(LoopClock clock);

        /** Stop outputs and resources owned by this realization. */
        void stop();
    }

    /** Adds read-only rows to the program's one FTC telemetry frame. */
    @FunctionalInterface
    public interface Presenter {

        /**
         * Add rows without clearing or committing telemetry and without advancing robot state.
         *
         * <p>Presenters run after configuration, in every INIT loop, and after active outputs. The
         * framework calls {@link Telemetry#update()} exactly once after all presenters return.</p>
         *
         * @param clock shared program clock for the current frame
         * @param telemetry additive FTC telemetry destination
         */
        void present(LoopClock clock, Telemetry telemetry);
    }

    private enum State {
        CONFIGURING,
        READY,
        STARTING,
        ACTIVE,
        TERMINAL
    }

    private final Telemetry telemetry;
    private final LoopClock clock = new LoopClock();
    private final Bindings bindings = new Bindings();
    private final TaskRunner taskRunner = new TaskRunner();
    private final BindingRegistrar bindingRegistrar = new ConfigurationBindingRegistrar();
    private final TaskBindings taskBindings = TaskBindings.of(bindingRegistrar, taskRunner);
    private final List<Service> services = new ArrayList<>();
    private final List<Output> outputs = new ArrayList<>();
    private final List<Presenter> presenters = new ArrayList<>();
    private final IdentityHashMap<Object, String> registrations = new IdentityHashMap<>();

    private State state = State.CONFIGURING;
    private Task rootTask;
    private boolean driveDeclared;
    private boolean bindingsUpdating;
    private boolean bindingCleanupPending;

    /** Framework-only construction; ordinary robot code receives the retained instance. */
    RobotProgram(Telemetry telemetry) {
        this.telemetry = Objects.requireNonNull(telemetry, "telemetry is required");
    }

    /**
     * Return the registration-only input surface backed by this program's one binding graph.
     *
     * <p>Retaining this view is safe, but every declaration after
     * {@link FtcRobotOpMode#configure(RobotProgram)} returns fails with an actionable lifecycle
     * error. The view deliberately exposes neither the binding heartbeat nor clearing.</p>
     */
    public BindingRegistrar bindings() {
        return bindingRegistrar;
    }

    /**
     * Return the Task-binding adapter backed by this program's binding graph and Task runner.
     *
     * <p>Task factories must still return a fresh single-use Task for every accepted event. The
     * framework advances the shared bindings before the shared runner in the same active cycle.</p>
     */
    public TaskBindings taskBindings() {
        return taskBindings;
    }

    /**
     * Declare an upstream service and return that exact object.
     *
     * @param service service to retain immediately
     * @param <T> concrete service type
     * @return the exact supplied service
     * @throws NullPointerException if {@code service} is null
     * @throws IllegalStateException if configuration has ended or this object identity is already
     *                               registered
     */
    public <T extends Service> T service(T service) {
        requireConfiguring("declare a service");
        T required = Objects.requireNonNull(service, "service is required");
        reserveIdentity(required, "service");
        services.add(required);
        return required;
    }

    /**
     * Declare a downstream output and return that exact object.
     *
     * <p>Register the mechanism or subsystem that privately owns its Plants, not a raw Plant.</p>
     *
     * @param output realization owner to retain immediately
     * @param <T> concrete output type
     * @return the exact supplied output
     * @throws NullPointerException if {@code output} is null
     * @throws IllegalStateException if configuration has ended or this object identity is already
     *                               registered
     */
    public <T extends Output> T output(T output) {
        requireConfiguring("declare an output");
        T required = Objects.requireNonNull(output, "output is required");
        reserveIdentity(required, "output");
        outputs.add(required);
        return required;
    }

    /**
     * Declare the program's one source-driven final drive output.
     *
     * <p>At its place in output declaration order, the program calls
     * {@link DriveCommandSink#update(LoopClock)}, samples and validates finite {@code source}
     * components, clamps them, then submits that one command. Cleanup calls
     * {@link DriveCommandSink#stop()} immediately, including when configuration fails before the
     * first drive update.</p>
     *
     * @param source final composed robot-centric drive source
     * @param sink final drive command owner
     * @param <T> concrete sink type
     * @return the exact supplied sink
     * @throws NullPointerException if an argument is null
     * @throws IllegalStateException if configuration has ended, a drive was already declared, or
     *                               the sink identity is already registered
     */
    public <T extends DriveCommandSink> T drive(DriveSource source, T sink) {
        requireConfiguring("declare drive");
        DriveSource requiredSource = Objects.requireNonNull(source, "drive source is required");
        T requiredSink = Objects.requireNonNull(sink, "drive sink is required");
        if (driveDeclared) {
            throw new IllegalStateException(
                    "RobotProgram already has a drive declaration; compose one final DriveSource "
                            + "and declare one final DriveCommandSink");
        }
        requireIdentityAvailable(requiredSink, "drive sink");

        driveDeclared = true;
        registrations.put(requiredSink, "drive sink");
        outputs.add(new SourceDrivenDriveOutput(requiredSource, requiredSink));
        return requiredSink;
    }

    /**
     * Declare the optional one-shot root Task for this program.
     *
     * <p>The framework starts it through the private runner at the exact FTC START clock boundary,
     * advances it after bindings in each active cycle, and actively cancels it during shutdown.</p>
     *
     * @param task fresh single-use root Task
     * @throws NullPointerException if {@code task} is null
     * @throws IllegalStateException if configuration has ended, a root already exists, or this
     *                               identity is already registered
     */
    public void rootTask(Task task) {
        requireConfiguring("declare a root Task");
        Task required = Objects.requireNonNull(task, "root task is required");
        if (rootTask != null) {
            throw new IllegalStateException(
                    "RobotProgram already has a root Task; compose one root with Tasks.sequence, "
                            + "Tasks.parallelAll, or Tasks.parallelDeadline");
        }
        reserveIdentity(required, "root Task");
        rootTask = required;
    }

    /**
     * Declare an additive presenter.
     *
     * @param presenter read-only frame contributor
     * @throws NullPointerException if {@code presenter} is null
     * @throws IllegalStateException if configuration has ended or this object identity is already
     *                               registered
     */
    public void presenter(Presenter presenter) {
        requireConfiguring("declare a presenter");
        Presenter required = Objects.requireNonNull(presenter, "presenter is required");
        reserveIdentity(required, "presenter");
        presenters.add(required);
    }

    /** Initialize the shared clock before robot declarations can inspect FTC-owned resources. */
    void beginInit(double runtimeSec) {
        requireState(State.CONFIGURING, "begin INIT");
        clock.reset(requireFiniteRuntime(runtimeSec));
    }

    /** Freeze the declaration graph after the one configuration callback returns. */
    void finishConfiguration() {
        requireState(State.CONFIGURING, "finish configuration");
        state = State.READY;
    }

    /** Render and commit the first presenter-only INIT frame. */
    void presentConfiguredInit() {
        requireState(State.READY, "present INIT telemetry");
        presentAndCommit(State.READY);
    }

    /** Advance the INIT clock and render presenters without advancing behavior or outputs. */
    void initLoop(double runtimeSec) {
        requireState(State.READY, "run INIT loop");
        clock.update(requireFiniteRuntime(runtimeSec));
        presentAndCommit(State.READY);
    }

    /** Start services and the optional root Task at the exact FTC START boundary. */
    void start(double runtimeSec) {
        requireState(State.READY, "start");
        clock.reset(requireFiniteRuntime(runtimeSec));
        state = State.STARTING;

        for (int index = 0; index < services.size(); index++) {
            services.get(index).start(clock);
            if (state != State.STARTING) {
                return;
            }
        }

        if (rootTask != null) {
            taskRunner.enqueue(rootTask);
            taskRunner.update(clock);
            if (state != State.STARTING) {
                return;
            }
        }

        // Realize exact-start Task requests once. This keeps every positive-duration request
        // observable by its downstream output even if the first FTC loop arrives after its time
        // boundary, and still performs no actuation during INIT.
        updateOutputs(State.STARTING);
        if (state == State.STARTING) {
            state = State.ACTIVE;
        }
    }

    /** Advance one complete active program cycle in the fixed documented order. */
    void loop(double runtimeSec) {
        requireState(State.ACTIVE, "run active loop");
        clock.update(requireFiniteRuntime(runtimeSec));

        for (int index = 0; index < services.size(); index++) {
            services.get(index).update(clock);
            if (state != State.ACTIVE) {
                return;
            }
        }

        RuntimeException bindingFailure = null;
        bindingsUpdating = true;
        try {
            bindings.update(clock);
        } catch (RuntimeException failure) {
            bindingFailure = failure;
        } finally {
            bindingsUpdating = false;
        }

        boolean cooperativeStop = bindingFailure instanceof BindingTraversalStopped
                && state == State.TERMINAL;
        RuntimeException completedBindingFailure = finishDeferredBindingCleanup(
                cooperativeStop ? null : bindingFailure
        );
        if (completedBindingFailure != null) {
            throw completedBindingFailure;
        }
        if (cooperativeStop || state != State.ACTIVE) {
            return;
        }

        taskRunner.update(clock);
        if (state != State.ACTIVE) {
            return;
        }

        updateOutputs(State.ACTIVE);
        if (state != State.ACTIVE) {
            return;
        }

        presentAndCommit(State.ACTIVE);
    }

    /** Mark terminal first, then attempt every cleanup action in its fixed ownership order. */
    void stop() {
        boolean abortBindingTraversal = bindingsUpdating && state != State.TERMINAL;
        Runnable[] actions = claimCleanupActions();
        if (actions.length != 0) {
            CleanupActions.attemptAll(actions);
        }
        if (abortBindingTraversal) {
            throw new BindingTraversalStopped();
        }
    }

    /** Preserve a lifecycle failure while attaching every later cleanup failure. */
    RuntimeException stopAfterFailure(RuntimeException primaryFailure) {
        RuntimeException required = Objects.requireNonNull(
                primaryFailure,
                "primaryFailure is required"
        );
        Runnable[] actions = claimCleanupActions();
        if (actions.length == 0) {
            return required;
        }
        return CleanupActions.attemptAllAfterFailure(required, actions);
    }

    /** Return whether this program has already claimed its one cleanup pass. */
    boolean isTerminal() {
        return state == State.TERMINAL;
    }

    private void updateOutputs(State expectedState) {
        for (int index = 0; index < outputs.size(); index++) {
            outputs.get(index).update(clock);
            if (state != expectedState) {
                return;
            }
        }
    }

    private void presentAndCommit(State expectedState) {
        for (int index = 0; index < presenters.size(); index++) {
            presenters.get(index).present(clock, telemetry);
            if (state != expectedState) {
                return;
            }
        }
        telemetry.update();
    }

    /**
     * Claim cleanup exactly once and snapshot callbacks after publishing terminal state.
     */
    private Runnable[] claimCleanupActions() {
        if (state == State.TERMINAL) {
            return new Runnable[0];
        }
        state = State.TERMINAL;

        if (bindingsUpdating) {
            // A binding participant will throw the private abort token after this stop returns.
            // Defer the whole cleanup sequence so Bindings.clear remains first structurally and
            // every cleanup failure retains the same primary/suppression order as ordinary STOP.
            bindingCleanupPending = true;
            return new Runnable[0];
        }
        return cleanupActions();
    }

    /** Snapshot the one fixed cleanup sequence after no binding traversal is active. */
    private Runnable[] cleanupActions() {
        List<Runnable> actions = new ArrayList<>();
        actions.add(taskRunner::cancelAndClear);
        actions.add(bindings::clear);
        for (int index = 0; index < outputs.size(); index++) {
            Output output = outputs.get(index);
            actions.add(output::stop);
        }
        for (int index = services.size() - 1; index >= 0; index--) {
            Service service = services.get(index);
            actions.add(service::stop);
        }
        return actions.toArray(new Runnable[0]);
    }

    /** Run cleanup after a reentrant STOP has unwound the protected binding traversal. */
    private RuntimeException finishDeferredBindingCleanup(RuntimeException primaryFailure) {
        if (!bindingCleanupPending) {
            return primaryFailure;
        }
        bindingCleanupPending = false;
        Runnable[] actions = cleanupActions();
        if (primaryFailure == null) {
            try {
                CleanupActions.attemptAll(actions);
                return null;
            } catch (RuntimeException cleanupFailure) {
                return cleanupFailure;
            }
        }
        return CleanupActions.attemptAllAfterFailure(primaryFailure, actions);
    }

    private void reserveIdentity(Object candidate, String role) {
        requireIdentityAvailable(candidate, role);
        registrations.put(candidate, role);
    }

    private void requireIdentityAvailable(Object candidate, String role) {
        String existingRole = registrations.get(candidate);
        if (existingRole != null) {
            throw new IllegalStateException(
                    "RobotProgram already registered this exact object as " + existingRole
                            + "; create a distinct " + role
                            + " or keep the object's complete lifecycle in one declared role");
        }
    }

    private void requireConfiguring(String operation) {
        if (state != State.CONFIGURING) {
            throw new IllegalStateException(
                    "RobotProgram cannot " + operation + " after configure(program) returns; "
                            + "declare the complete graph inside that one configuration callback");
        }
    }

    private void requireState(State required, String operation) {
        if (state != required) {
            throw new IllegalStateException(
                    "RobotProgram cannot " + operation + " while its state is " + state
                            + "; FtcRobotOpMode owns this lifecycle");
        }
    }

    private static double requireFiniteRuntime(double runtimeSec) {
        if (!Double.isFinite(runtimeSec)) {
            throw new IllegalArgumentException(
                    "FTC runtimeSec must be finite, got " + runtimeSec);
        }
        return runtimeSec;
    }

    /** Abort token used only to unwind a binding traversal after cooperative reentrant STOP. */
    private static final class BindingTraversalStopped extends RuntimeException {
        private BindingTraversalStopped() {
            super(null, null, false, false);
        }
    }

    /** Binding view that keeps every declaration inside the one configuration boundary. */
    private final class ConfigurationBindingRegistrar implements BindingRegistrar {

        @Override
        public void onRise(BooleanSource signal, Runnable action) {
            requireConfiguring("register a binding");
            bindings.onRise(guardBindingSource(signal), guardBindingAction(action));
        }

        @Override
        public void onFall(BooleanSource signal, Runnable action) {
            requireConfiguring("register a binding");
            bindings.onFall(guardBindingSource(signal), guardBindingAction(action));
        }

        @Override
        public void mirrorOnChange(BooleanSource signal, Consumer<Boolean> consumer) {
            requireConfiguring("register a binding");
            bindings.mirrorOnChange(
                    guardBindingSource(signal),
                    guardBindingConsumer(consumer)
            );
        }

        @Override
        public void whileHigh(BooleanSource signal, Runnable action) {
            requireConfiguring("register a binding");
            bindings.whileHigh(guardBindingSource(signal), guardBindingAction(action));
        }

        @Override
        public void whileLow(BooleanSource signal, Runnable action) {
            requireConfiguring("register a binding");
            bindings.whileLow(guardBindingSource(signal), guardBindingAction(action));
        }

        @Override
        public void toggleOnRise(
                BooleanSource signal,
                Runnable onEnabled,
                Runnable onDisabled
        ) {
            requireConfiguring("register a binding");
            bindings.toggleOnRise(
                    guardBindingSource(signal),
                    guardBindingAction(onEnabled),
                    guardBindingAction(onDisabled)
            );
        }

        @Override
        public void toggleOnRise(BooleanSource signal, Consumer<Boolean> consumer) {
            requireConfiguring("register a binding");
            bindings.toggleOnRise(
                    guardBindingSource(signal),
                    guardBindingConsumer(consumer)
            );
        }

        @Override
        public void nudgeOnRise(
                BooleanSource increaseSignal,
                BooleanSource decreaseSignal,
                double step,
                DoubleConsumer adjuster
        ) {
            requireConfiguring("register a binding");
            bindings.nudgeOnRise(
                    guardBindingSource(increaseSignal),
                    guardBindingSource(decreaseSignal),
                    step,
                    guardBindingConsumer(adjuster)
            );
        }

        @Override
        public void copyEachCycle(ScalarSource source, DoubleConsumer consumer) {
            requireConfiguring("register a binding");
            bindings.copyEachCycle(
                    guardBindingSource(source),
                    guardBindingConsumer(consumer)
            );
        }

        private BooleanSource guardBindingSource(BooleanSource source) {
            BooleanSource required = Objects.requireNonNull(source, "signal is required");
            return clock -> {
                requireActiveBindingParticipant();
                boolean value = required.getAsBoolean(clock);
                requireActiveBindingParticipant();
                return value;
            };
        }

        private ScalarSource guardBindingSource(ScalarSource source) {
            ScalarSource required = Objects.requireNonNull(source, "source is required");
            return clock -> {
                requireActiveBindingParticipant();
                double value = required.getAsDouble(clock);
                requireActiveBindingParticipant();
                return value;
            };
        }

        private Runnable guardBindingAction(Runnable action) {
            Runnable required = Objects.requireNonNull(action, "action is required");
            return () -> {
                requireActiveBindingParticipant();
                required.run();
                requireActiveBindingParticipant();
            };
        }

        private Consumer<Boolean> guardBindingConsumer(Consumer<Boolean> consumer) {
            Consumer<Boolean> required = Objects.requireNonNull(consumer, "consumer is required");
            return value -> {
                requireActiveBindingParticipant();
                required.accept(value);
                requireActiveBindingParticipant();
            };
        }

        private DoubleConsumer guardBindingConsumer(DoubleConsumer consumer) {
            DoubleConsumer required = Objects.requireNonNull(consumer, "consumer is required");
            return value -> {
                requireActiveBindingParticipant();
                required.accept(value);
                requireActiveBindingParticipant();
            };
        }

        private void requireActiveBindingParticipant() {
            if (state != State.ACTIVE) {
                throw new BindingTraversalStopped();
            }
        }
    }

    /** Source/sink pair that joins the ordinary output declaration order as one owner. */
    private final class SourceDrivenDriveOutput implements Output {
        private final DriveSource source;
        private final DriveCommandSink sink;

        private SourceDrivenDriveOutput(DriveSource source, DriveCommandSink sink) {
            this.source = source;
            this.sink = sink;
        }

        @Override
        public void update(LoopClock clock) {
            State expectedState = state;
            sink.update(clock);
            if (state != expectedState) {
                return;
            }

            DriveSignal signal = source.get(clock);
            if (state != expectedState) {
                return;
            }
            sink.drive(requireFiniteDriveSignal(Objects.requireNonNull(
                    signal,
                    "drive source returned null; return a DriveSignal for every active cycle"
            )).clamped());
        }

        @Override
        public void stop() {
            sink.stop();
        }

        private DriveSignal requireFiniteDriveSignal(DriveSignal signal) {
            if (!Double.isFinite(signal.axial)
                    || !Double.isFinite(signal.lateral)
                    || !Double.isFinite(signal.omega)) {
                throw new IllegalArgumentException(
                        "drive source must return finite axial, lateral, and omega components "
                                + "before clamping, got " + signal
                );
            }
            return signal;
        }
    }
}
