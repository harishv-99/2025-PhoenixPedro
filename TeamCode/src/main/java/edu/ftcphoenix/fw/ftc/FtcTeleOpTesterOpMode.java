package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
 */
public abstract class FtcTeleOpTesterOpMode extends OpMode {

    private final LoopClock clock = new LoopClock();

    private TesterContext ctx;
    private TeleOpTester tester;

    /**
     * Return the tester to run. Most commonly this is a {@code TesterSuite} that
     * registers multiple testers for selection from a menu.
     */
    protected abstract TeleOpTester createTester();

    /**
     * {@inheritDoc}
     */
    @Override
    public final void init() {

        // Start dt tracking immediately so tester init/init_loop can rely on a "started" clock.
        clock.reset(getRuntime());

        // Build shared tester context (includes the shared loop clock).
        ctx = new TesterContext(hardwareMap, telemetry, gamepad1, gamepad2, clock);

        tester = createTester();
        if (tester == null) {
            telemetry.addLine("ERROR: createTester() returned null.");
            telemetry.update();
            return;
        }

        tester.init(ctx);

        telemetry.addLine("Ready: " + tester.name());
        telemetry.update();
    }

    /** {@inheritDoc} */
    @Override
    public final void init_loop() {
        if (tester == null) return;

        clock.update(getRuntime());
        tester.initLoop(clock.dtSec());
    }

    /** {@inheritDoc} */
    @Override
    public final void start() {
        if (tester == null) return;

        // Reset dt at transition to RUNNING so loop dt is clean.
        clock.reset(getRuntime());
        tester.start();
    }

    /** {@inheritDoc} */
    @Override
    public final void loop() {
        if (tester == null) return;

        clock.update(getRuntime());
        tester.loop(clock.dtSec());
    }

    /** {@inheritDoc} */
    @Override
    public final void stop() {
        if (tester == null) return;
        tester.stop();
    }
}
