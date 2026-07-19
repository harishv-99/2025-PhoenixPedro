package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterContext;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;

/**
 * FTC SDK {@link OpMode} base class for running a Phoenix {@link TeleOpTester}.
 *
 * <p>This lets testers live outside TeamCode (e.g., in {@code edu.ftcphoenix.robots...})
 * while TeamCode provides only a tiny wrapper OpMode that returns a tester (usually a
 * {@link TesterSuite}).</p>
 *
 * <h2>Lifecycle mapping</h2>
 * <ul>
 *   <li>{@link #init()} → {@link TeleOpTester#init(TesterContext)}</li>
 *   <li>{@link #init_loop()} → {@link TeleOpTester#initLoop(double)}</li>
 *   <li>{@link #start()} → {@link TeleOpTester#start()}</li>
 *   <li>{@link #loop()} → {@link TeleOpTester#loop(double)} with {@code dtSec}</li>
 *   <li>{@link #stop()} → {@link TeleOpTester#stop()}</li>
 * </ul>
 *
 * <h2>One loop, one heartbeat</h2>
 * <p>This OpMode owns a single {@link LoopClock} instance and advances it exactly once per FTC loop
 * (in {@link #init_loop()} and {@link #loop()}). That shared clock is also passed into
 * {@link TesterContext} so that per-cycle systems (like button edges and bindings) can be idempotent
 * by {@link LoopClock#cycle()} across nested callers (suite → active tester).</p>
 *
 * <h2>Fail-stop ownership</h2>
 * <p>The tester returned by {@link #createTester()} is retained before its
 * {@link TeleOpTester#init(TesterContext)} callback begins. If a tester lifecycle callback throws
 * a {@link RuntimeException}, this owner becomes terminal, detaches the tester, attempts
 * {@link TeleOpTester#stop()} exactly once, and rethrows the original failure. A cleanup failure is
 * attached to the original failure as a suppressed exception. Later FTC lifecycle callbacks do
 * nothing. {@link Error Errors} are not caught.</p>
 */
public abstract class FtcTeleOpTesterOpMode extends OpMode {

    private final LoopClock clock = new LoopClock();

    private TesterContext ctx;
    private TeleOpTester tester;
    private boolean initAttempted;
    private boolean terminal;

    /**
     * Return the tester to run. Most commonly this is a {@code TesterSuite} that
     * registers multiple testers for selection from a menu.
     *
     * <p>Construct and return an inactive tester. Hardware and other resources that require
     * cleanup belong in {@link TeleOpTester#init(TesterContext)}, after this owner can retain the
     * tester for fail-stop cleanup.</p>
     */
    protected abstract TeleOpTester createTester();

    /**
     * {@inheritDoc}
     */
    @Override
    public final void init() {
        if (terminal) {
            return;
        }
        if (initAttempted) {
            throw new IllegalStateException(
                    "FtcTeleOpTesterOpMode.init() may be called only once per OpMode instance");
        }
        initAttempted = true;

        // Start dt tracking immediately so tester init/init_loop can rely on a "started" clock.
        clock.reset(getRuntime());

        // Build shared tester context (includes the shared loop clock).
        ctx = new TesterContext(hardwareMap, telemetry, gamepad1, gamepad2, clock);

        TeleOpTester created;
        try {
            created = createTester();
        } catch (RuntimeException failure) {
            terminal = true;
            reportCreationFailure(failure);
            throw failure;
        }

        // A custom factory may reenter STOP. STOP wins; the returned tester is still inactive and
        // ownership never transfers into this already-terminal host.
        if (terminal) {
            return;
        }

        if (created == null) {
            terminal = true;
            IllegalStateException failure = new IllegalStateException(
                    "createTester() returned null; return a configured TeleOpTester instance");
            reportCreationFailure(failure);
            throw failure;
        }

        // Retain ownership before any lifecycle callback can acquire resources or fail.
        tester = created;
        try {
            created.init(ctx);
            if (terminal || tester != created) {
                return;
            }

            String testerName = created.name();
            if (terminal || tester != created) {
                return;
            }
            telemetry.addLine("Ready: " + testerName);
            if (terminal || tester != created) {
                return;
            }
            telemetry.update();
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** {@inheritDoc} */
    @Override
    public final void init_loop() {
        TeleOpTester active = tester;
        if (terminal || active == null) return;

        try {
            clock.update(getRuntime());
            active.initLoop(clock.dtSec());
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** {@inheritDoc} */
    @Override
    public final void start() {
        TeleOpTester active = tester;
        if (terminal || active == null) return;

        try {
            // Reset dt at transition to RUNNING so loop dt is clean.
            clock.reset(getRuntime());
            active.start();
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** {@inheritDoc} */
    @Override
    public final void loop() {
        TeleOpTester active = tester;
        if (terminal || active == null) return;

        try {
            clock.update(getRuntime());
            active.loop(clock.dtSec());
        } catch (RuntimeException failure) {
            terminateAfterFailure(failure);
        }
    }

    /** {@inheritDoc} */
    @Override
    public final void stop() {
        TeleOpTester active = detachAndTerminate();
        if (active != null) {
            active.stop();
        }
    }

    private TeleOpTester detachAndTerminate() {
        terminal = true;
        TeleOpTester active = tester;
        tester = null;
        return active;
    }

    private void terminateAfterFailure(RuntimeException failure) {
        TeleOpTester active = detachAndTerminate();
        if (active != null) {
            throw CleanupActions.attemptAllAfterFailure(failure, active::stop);
        }
        throw failure;
    }

    private void reportCreationFailure(RuntimeException failure) {
        try {
            telemetry.addLine(
                    "ERROR: createTester() failed: " + actionableMessage(failure));
            telemetry.addLine(
                    "Check tester construction; acquire hardware and resources in init().");
            telemetry.update();
        } catch (RuntimeException telemetryFailure) {
            if (failure != telemetryFailure) {
                failure.addSuppressed(telemetryFailure);
            }
        }
    }

    private static String actionableMessage(RuntimeException failure) {
        String message = failure.getMessage();
        if (message == null || message.trim().isEmpty()) {
            return failure.getClass().getSimpleName();
        }
        return message;
    }
}
