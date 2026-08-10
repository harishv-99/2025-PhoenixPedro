package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
 *   <li>{@link #init()} → console sample → {@link TeleOpTester#init(TesterContext)}</li>
 *   <li>{@link #init_loop()} → clock update → console sample →
 *       {@link TeleOpTester#initLoop(double)}</li>
 *   <li>{@link #start()} → clock reset → console sample → {@link TeleOpTester#start()}</li>
 *   <li>{@link #loop()} → clock update → console sample →
 *       {@link TeleOpTester#loop(double)} with {@code dtSec}</li>
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
 * <p>An initial console sample occurs while the returned tester is still inactive. Failure there
 * terminalizes this host without calling {@link TeleOpTester#stop()}. After that sample succeeds,
 * the tester is retained before its {@link TeleOpTester#init(TesterContext)} callback begins. If a
 * tester callback or later console sample throws a {@link RuntimeException}, this owner becomes
 * terminal, detaches the tester, attempts {@link TeleOpTester#stop()} exactly once, and rethrows the
 * original failure. A cleanup failure is attached to the original failure as a suppressed
 * exception. Later FTC lifecycle callbacks do nothing. {@link Error Errors} are not caught.</p>
 */
public abstract class FtcTeleOpTesterOpMode extends OpMode {

    /**
     * Stable telemetry and input objects owned by this tester host.
     *
     * <p>The host creates one console during {@link #init()}, samples it immediately before tester
     * {@code init}, {@code initLoop}, {@code start}, and {@code loop}, and passes the same object
     * identities through {@link TesterContext}. STOP is cleanup and deliberately performs no input
     * sample. A console that reads snapshots from another transport must copy each snapshot into
     * its stable {@link Gamepad} objects in {@link #sampleInputs()}.</p>
     *
     * <p>This is an advanced host-extension seam. Ordinary testers consume only
     * {@link TesterContext}; they do not select or inspect their transport.</p>
     */
    protected interface TesterConsole {

        /** Telemetry sink used by the tester and the host's ready message. */
        Telemetry telemetry();

        /** Stable gamepad-one object. */
        Gamepad gamepad1();

        /** Stable gamepad-two object. */
        Gamepad gamepad2();

        /**
         * Refreshes the stable input objects before the next tester callback.
         *
         * <p>Throw a {@link RuntimeException} when the selected input transport cannot provide a
         * trustworthy sample. An initial failure prevents tester initialization; a later failure
         * terminally fail-stops the retained tester.</p>
         */
        void sampleInputs();
    }

    private final LoopClock clock = new LoopClock();

    private TesterContext ctx;
    private TesterConsole console;
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
     * Creates the telemetry/input console used for this OpMode instance.
     *
     * <p>The default console uses the FTC OpMode's Driver Station telemetry and physical gamepads,
     * and its input sample is a no-op because the SDK keeps those gamepad objects current. Override
     * only for a materially different host transport. Construct an inactive adapter here; acquire
     * no hardware or resource that requires cleanup.</p>
     */
    protected TesterConsole createTesterConsole() {
        return new DirectTesterConsole(telemetry, gamepad1, gamepad2);
    }

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

        TesterConsole createdConsole;
        try {
            createdConsole = requireValidConsole(createTesterConsole());
        } catch (RuntimeException failure) {
            terminal = true;
            reportCreationFailure("createTesterConsole()", failure);
            throw failure;
        }

        // A custom console factory may reenter STOP. Its inactive adapter owns no cleanup.
        if (terminal) {
            return;
        }

        console = createdConsole;
        // Build shared tester context with stable console objects and the one shared loop clock.
        ctx = new TesterContext(
                hardwareMap,
                createdConsole.telemetry(),
                createdConsole.gamepad1(),
                createdConsole.gamepad2(),
                clock);

        TeleOpTester created;
        try {
            created = createTester();
        } catch (RuntimeException failure) {
            terminal = true;
            reportCreationFailure("createTester()", failure);
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
            reportCreationFailure("createTester()", failure);
            throw failure;
        }

        try {
            createdConsole.sampleInputs();
        } catch (RuntimeException failure) {
            terminal = true;
            throw failure;
        }
        if (terminal) {
            return;
        }

        // Retain ownership before any tester lifecycle callback can acquire resources or fail.
        tester = created;
        try {
            created.init(ctx);
            if (terminal || tester != created) return;

            String testerName = created.name();
            if (terminal || tester != created) return;

            ctx.telemetry.addLine("Ready: " + testerName);
            if (terminal || tester != created) return;

            ctx.telemetry.update();
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
            console.sampleInputs();
            if (terminal || tester != active) return;
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
            console.sampleInputs();
            if (terminal || tester != active) return;
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
            console.sampleInputs();
            if (terminal || tester != active) return;
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

    private void reportCreationFailure(String operation, RuntimeException failure) {
        try {
            telemetry.addLine(
                    "ERROR: " + operation + " failed: " + actionableMessage(failure));
            if ("createTester()".equals(operation)) {
                telemetry.addLine(
                        "Check tester construction; acquire hardware and resources in init().");
            } else {
                telemetry.addLine(
                        "Check tester-console configuration; acquire no hardware in its factory.");
            }
            telemetry.update();
        } catch (RuntimeException telemetryFailure) {
            if (failure != telemetryFailure) {
                failure.addSuppressed(telemetryFailure);
            }
        }
    }

    private static TesterConsole requireValidConsole(TesterConsole console) {
        if (console == null) {
            throw new IllegalStateException(
                    "createTesterConsole() returned null; return a configured TesterConsole");
        }
        Telemetry telemetry = console.telemetry();
        Gamepad gamepad1 = console.gamepad1();
        Gamepad gamepad2 = console.gamepad2();
        if (telemetry == null) {
            throw new IllegalStateException("TesterConsole.telemetry() returned null");
        }
        if (gamepad1 == null) {
            throw new IllegalStateException("TesterConsole.gamepad1() returned null");
        }
        if (gamepad2 == null) {
            throw new IllegalStateException("TesterConsole.gamepad2() returned null");
        }
        return new ValidatedTesterConsole(console, telemetry, gamepad1, gamepad2);
    }

    private static String actionableMessage(RuntimeException failure) {
        String message = failure.getMessage();
        if (message == null || message.trim().isEmpty()) {
            return failure.getClass().getSimpleName();
        }
        return message;
    }

    private static final class DirectTesterConsole implements TesterConsole {
        private final Telemetry telemetry;
        private final Gamepad gamepad1;
        private final Gamepad gamepad2;

        private DirectTesterConsole(
                Telemetry telemetry,
                Gamepad gamepad1,
                Gamepad gamepad2
        ) {
            this.telemetry = telemetry;
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
        }

        @Override
        public Telemetry telemetry() {
            return telemetry;
        }

        @Override
        public Gamepad gamepad1() {
            return gamepad1;
        }

        @Override
        public Gamepad gamepad2() {
            return gamepad2;
        }

        @Override
        public void sampleInputs() {
            // FTC mutates the stable OpMode gamepad objects directly.
        }
    }

    private static final class ValidatedTesterConsole implements TesterConsole {
        private final TesterConsole delegate;
        private final Telemetry telemetry;
        private final Gamepad gamepad1;
        private final Gamepad gamepad2;

        private ValidatedTesterConsole(
                TesterConsole delegate,
                Telemetry telemetry,
                Gamepad gamepad1,
                Gamepad gamepad2
        ) {
            this.delegate = delegate;
            this.telemetry = telemetry;
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
        }

        @Override
        public Telemetry telemetry() {
            return telemetry;
        }

        @Override
        public Gamepad gamepad1() {
            return gamepad1;
        }

        @Override
        public Gamepad gamepad2() {
            return gamepad2;
        }

        @Override
        public void sampleInputs() {
            delegate.sampleInputs();
        }
    }
}
