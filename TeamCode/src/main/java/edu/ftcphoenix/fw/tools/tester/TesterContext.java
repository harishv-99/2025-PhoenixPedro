package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Shared runtime context for Phoenix testers.
 *
 * <p>This is intentionally small and FTC-centric: it provides access to the FTC SDK
 * objects that testers commonly need (hardware map, telemetry, and gamepads).</p>
 *
 * <h2>One loop, one heartbeat</h2>
 * <p>The {@link #clock} provided here is the single per-OpMode loop heartbeat.
 * Testers should treat it as read-only and should <b>not</b> create their own
 * independent clocks for per-cycle systems (like button edge tracking).</p>
 *
 * <p>Using a shared {@link LoopClock} enables per-cycle idempotency: if input systems
 * are updated multiple times in the same OpMode cycle, they can safely treat the
 * second call as a no-op using {@link LoopClock#cycle()}.</p>
 */
public final class TesterContext {

    /**
     * FTC hardware map.
     */
    public final HardwareMap hw;

    /**
     * FTC telemetry (typically the Driver Hub telemetry).
     */
    public final Telemetry telemetry;

    /**
     * FTC gamepad 1.
     */
    public final Gamepad gamepad1;

    /**
     * FTC gamepad 2.
     */
    public final Gamepad gamepad2;

    /**
     * Per-loop heartbeat owned by the OpMode that is running the tester.
     *
     * <p>This clock is advanced once per OpMode cycle by the runner. Testers should
     * use it for dt and for per-cycle identity via {@link LoopClock#cycle()}.</p>
     */
    public final LoopClock clock;

    /**
     * Create a tester context.
     *
     * @param hw        FTC hardware map
     * @param telemetry FTC telemetry sink
     * @param gamepad1  FTC gamepad 1
     * @param gamepad2  FTC gamepad 2
     * @param clock     shared per-loop clock advanced by the tester runner
     */
    public TesterContext(
            HardwareMap hw,
            Telemetry telemetry,
            Gamepad gamepad1,
            Gamepad gamepad2,
            LoopClock clock
    ) {
        this.hw = hw;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.clock = clock;
    }
}
