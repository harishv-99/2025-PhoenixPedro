package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * FTC iterative OpMode host for one declarative Phoenix {@link RobotProgram}.
 *
 * <p>Ordinary robot code overrides only {@link #configure(RobotProgram)}. The final FTC callbacks
 * own the shared clock, optional prestart policy, exact START boundary, fixed active phase order,
 * one telemetry commit, and terminal fail-stop cleanup.</p>
 *
 * <pre>{@code
 * public final class ExampleTeleOp extends FtcRobotOpMode {
 *     @Override
 *     protected void configure(RobotProgram program) {
 *         IntakeMechanism intake = program.output(
 *                 new IntakeMechanism(hardwareMap, IntakeConfig.current()));
 *         Controls controls = new Controls(gamepad1);
 *         controls.bind(program.callbackBindings(), intake);
 *         program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap));
 *         program.presenter((clock, telemetry) ->
 *                 telemetry.addData("intake.mode", intake.status().mode));
 *     }
 * }
 * }</pre>
 *
 * <p>If configuration or a runtime phase throws a {@link RuntimeException}, this host becomes
 * terminal, cancels Tasks, clears bindings, stops outputs, and stops services before rethrowing the
 * exact original failure. Cleanup failures are suppressed on that primary. An optional
 * {@link RobotProgram#stopHandoff(java.util.function.Supplier, java.util.function.Consumer,
 * Runnable) stop handoff} publishes only after a normal active STOP cleans up successfully.
 * Explicit, repeated, or reentrant STOP is idempotent. {@link Error Errors} are not caught.</p>
 */
public abstract class FtcRobotOpMode extends OpMode {

    private RobotProgram program;
    private boolean initAttempted;
    private boolean terminal;

    /**
     * Construct robot-specific owners and declare their roles exactly once during FTC INIT.
     *
     * <p>The supplied program is already retained by the host, so an owner registered before a
     * later construction failure participates in fail-stop cleanup. Make declarations only inside
     * this callback; the graph freezes when it returns.</p>
     *
     * @param program framework-created declaration surface
     */
    protected abstract void configure(RobotProgram program);

    /** Construct, freeze, update prestart policy, and present without active actuation. */
    @Override
    public final void init() {
        if (terminal) {
            return;
        }
        if (initAttempted) {
            throw new IllegalStateException(
                    "FtcRobotOpMode.init() may be called only once per OpMode instance");
        }
        initAttempted = true;

        RobotProgram created = new RobotProgram(telemetry);
        program = created;
        try {
            created.beginInit(getRuntime());
            configure(created);
            if (!stillOwns(created)) {
                return;
            }
            created.finishConfiguration();
            if (!stillOwns(created)) {
                return;
            }
            created.presentConfiguredInit();
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** Advance the clock and prestart policy, then present and commit one INIT frame. */
    @Override
    public final void init_loop() {
        RobotProgram active = activeProgram();
        if (active == null) {
            return;
        }
        try {
            active.initLoop(getRuntime());
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** Freeze prestart policy, then either block inertly or start active owners once. */
    @Override
    public final void start() {
        RobotProgram active = activeProgram();
        if (active == null) {
            return;
        }
        try {
            active.start(getRuntime());
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** Advance one active cycle, or one presenter-only frame while start is blocked. */
    @Override
    public final void loop() {
        RobotProgram active = activeProgram();
        if (active == null) {
            return;
        }
        try {
            active.loop(getRuntime());
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** Terminalize first, then attempt the program's one cleanup pass. */
    @Override
    public final void stop() {
        RobotProgram active = detachAndTerminate();
        if (active != null) {
            active.stop();
        }
    }

    private RobotProgram activeProgram() {
        if (terminal) {
            return null;
        }
        return program;
    }

    private boolean stillOwns(RobotProgram expected) {
        return !terminal && program == expected && !expected.isTerminal();
    }

    private RobotProgram detachAndTerminate() {
        terminal = true;
        RobotProgram active = program;
        program = null;
        return active;
    }

    private void terminateAfterFailure(RuntimeException failure) {
        RobotProgram active = detachAndTerminate();
        if (active != null) {
            throw active.stopAfterFailure(failure);
        }
        throw failure;
    }
}
