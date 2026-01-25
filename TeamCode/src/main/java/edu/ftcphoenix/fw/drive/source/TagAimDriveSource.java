package edu.ftcphoenix.fw.drive.source;

import edu.ftcphoenix.fw.debug.DebugSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.input.Button;
import edu.ftcphoenix.fw.sensing.BearingSource;
import edu.ftcphoenix.fw.sensing.BearingSource.BearingSample;
import edu.ftcphoenix.fw.sensing.TagAimController;
import edu.ftcphoenix.fw.util.LoopClock;

/**
 * {@link DriveSource} wrapper that adds tag-based auto-aim to an existing drive.
 *
 * <h2>Role</h2>
 * <p>
 * {@code TagAimDriveSource} is a reusable implementation of the same behavior
 * that a typical {@code TagAim.teleOpAim(...)} helper would provide:
 * </p>
 *
 * <ul>
 *   <li>It holds a "base" {@link DriveSource} (e.g., stick-based drive).</li>
 *   <li>It reads a {@link BearingSource} (e.g., AprilTags wrapped as bearing).</li>
 *   <li>It uses a {@link TagAimController} to turn bearing into {@code omega}.</li>
 *   <li>When the {@link Button} is pressed:
 *     <ul>
 *       <li>Axial and lateral commands come from the base source.</li>
 *       <li>Omega is overridden by the controller's output.</li>
 *     </ul>
 *   </li>
 *   <li>When the button is <b>not</b> pressed, it simply passes through the
 *       base {@link DriveSignal} unchanged.</li>
 * </ul>
 *
 * <p>
 * This class is useful if you prefer to construct and hold an explicit object
 * instead of using an anonymous wrapper or inline lambda.
 * </p>
 *
 * <h2>Sign conventions</h2>
 *
 * <p>
 * {@code TagAimDriveSource} does not change the meaning of {@link DriveSignal};
 * it only decides <em>who</em> supplies {@link DriveSignal#omega}:
 * </p>
 *
 * <ul>
 *   <li>When aiming is <b>off</b>, {@code omega} comes from {@code baseDrive}.</li>
 *   <li>When aiming is <b>on</b>, {@code omega} comes from {@link TagAimController}.</li>
 * </ul>
 *
 * <p>
 * The maintained sign conventions are those of {@link DriveSignal}:
 * </p>
 *
 * <ul>
 *   <li>{@code axial &gt; 0} &rarr; drive forward</li>
 *   <li>{@code lateral &gt; 0} &rarr; strafe right</li>
 *   <li>{@code omega &gt; 0} &rarr; rotate clockwise (turn right, viewed from above)</li>
 * </ul>
 *
 * <p>
 * {@link TagAimController} is responsible for choosing {@code omega} so that
 * the robot turns toward the target (e.g., tag left &rarr; {@code omega &lt; 0}
 * turn left; tag right &rarr; {@code omega &gt; 0} turn right).
 * </p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * Gamepads pads = Gamepads.create(gamepad1, gamepad2);
 *
 * // Base driver control (sticks) for mecanum drive.
 * DriveSource baseDrive = GamepadDriveSource.teleOpMecanumStandard(pads);
 *
 * // Bearing source (e.g., from AprilTag sensor).
 * BearingSource bearing = myTagSensor.asBearingSource();
 *
 * // TagAimController configured elsewhere (PID, deadband, maxOmega, loss policy).
 * TagAimController aimCtrl = new TagAimController(...);
 *
 * // Aim button (e.g., P1 left bumper).
 * Button aimButton = pads.p1().leftBumper();
 *
 * DriveSource aimedDrive = new TagAimDriveSource(
 *         baseDrive,
 *         aimButton,
 *         bearing,
 *         aimCtrl
 * );
 *
 * // In your loop:
 * clock.update(getRuntime());
 * DriveSignal signal = aimedDrive.get(clock).clamped();
 * drivebase.drive(signal);
 * drivebase.update(clock);
 * }</pre>
 */
public final class TagAimDriveSource implements DriveSource {

    private final DriveSource baseDrive;
    private final Button aimButton;
    private final BearingSource bearingSource;
    private final TagAimController controller;

    // Debug / introspection state
    private DriveSignal lastBaseSignal = DriveSignal.ZERO;
    private DriveSignal lastOutputSignal = DriveSignal.ZERO;
    private BearingSample lastBearingSample = null;
    private double lastOmegaFromController = 0.0;
    private boolean lastAimActive = false;

    /**
     * Construct a tag-aiming drive source.
     *
     * @param baseDrive     existing drive source (sticks, planner, etc.); must not be null
     * @param aimButton     button that enables aiming while pressed; must not be null
     * @param bearingSource source of bearing measurements to the target; must not be null
     * @param controller    controller that turns bearing into omega; must not be null
     */
    public TagAimDriveSource(DriveSource baseDrive,
                             Button aimButton,
                             BearingSource bearingSource,
                             TagAimController controller) {
        if (baseDrive == null) {
            throw new IllegalArgumentException("baseDrive is required");
        }
        if (aimButton == null) {
            throw new IllegalArgumentException("aimButton is required");
        }
        if (bearingSource == null) {
            throw new IllegalArgumentException("bearingSource is required");
        }
        if (controller == null) {
            throw new IllegalArgumentException("controller is required");
        }

        this.baseDrive = baseDrive;
        this.aimButton = aimButton;
        this.bearingSource = bearingSource;
        this.controller = controller;
    }

    /**
     * Get the current drive signal, optionally overriding omega to aim at the tag.
     *
     * <p>Behavior:</p>
     * <ul>
     *   <li>If {@code aimButton} is <b>not</b> pressed, returns the result of
     *       {@link DriveSource#get(LoopClock)} from {@code baseDrive}.</li>
     *   <li>If {@code aimButton} <b>is</b> pressed:
     *     <ul>
     *       <li>Sample bearing via {@link BearingSource#sample(LoopClock)}.</li>
     *       <li>Update the {@link TagAimController} via
     *           {@link TagAimController#update(LoopClock, BearingSample)}.</li>
     *       <li>Keep axial/lateral from the base drive, override omega using the
     *           controller's output.</li>
     *     </ul>
     *   </li>
     * </ul>
     *
     * <p>
     * The omega used when aiming is the controller's output and follows the
     * {@link DriveSignal} convention: {@code omega &gt; 0} rotates clockwise,
     * {@code omega &lt; 0} rotates counter-clockwise.
     * </p>
     */
    @Override
    public DriveSignal get(LoopClock clock) {
        // Get the base driver command first.
        DriveSignal base = baseDrive.get(clock);
        if (base == null) {
            base = DriveSignal.ZERO;
        }
        lastBaseSignal = base;

        if (!aimButton.isHeld()) {
            // No aiming: pass through base drive signal.
            lastAimActive = false;
            lastBearingSample = null;
            lastOmegaFromController = base.omega;
            lastOutputSignal = base;
            return base;
        }

        // Aiming: sample bearing and update the controller.
        BearingSample sample = bearingSource.sample(clock);
        lastBearingSample = sample;

        double omega = controller.update(clock, sample);
        lastOmegaFromController = omega;
        lastAimActive = true;

        // Preserve driver-controlled axial/lateral, override omega to aim.
        DriveSignal out = new DriveSignal(base.axial, base.lateral, omega);
        lastOutputSignal = out;
        return out;
    }

    // ------------------------------------------------------------------------
    // Debug support
    // ------------------------------------------------------------------------

    /**
     * Dump internal state to a {@link DebugSink}.
     *
     * <p>
     * This is intended for one-off debugging and tuning. Callers can choose
     * any prefix they like; nested callers often use dotted paths such as
     * {@code "drive.tagAim"}.
     * </p>
     *
     * <p>
     * This method is defensive: if {@code dbg} is {@code null}, it does
     * nothing. Framework classes consistently follow this pattern so callers
     * may freely pass a {@code NullDebugSink} or {@code null}.
     * </p>
     *
     * @param dbg    debug sink to write to (may be {@code null})
     * @param prefix key prefix for all entries (may be {@code null} or empty)
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagAim" : prefix;

        dbg.addLine(p + ": TagAimDriveSource");

        dbg.addData(p + ".aimActive", lastAimActive);

        if (lastBearingSample != null) {
            dbg.addData(p + ".bearing.hasTarget", lastBearingSample.hasTarget);
            dbg.addData(p + ".bearing.bearingRad", lastBearingSample.bearingRad);
        } else {
            dbg.addData(p + ".bearing.hasTarget", false);
        }

        dbg.addData(p + ".omega.fromController", lastOmegaFromController);

        dbg.addData(p + ".base.axial", lastBaseSignal.axial);
        dbg.addData(p + ".base.lateral", lastBaseSignal.lateral);
        dbg.addData(p + ".base.omega", lastBaseSignal.omega);

        dbg.addData(p + ".out.axial", lastOutputSignal.axial);
        dbg.addData(p + ".out.lateral", lastOutputSignal.lateral);
        dbg.addData(p + ".out.omega", lastOutputSignal.omega);
    }

    /**
     * Whether aiming was active the last time {@link #get(LoopClock)} was called.
     */
    public boolean isLastAimActive() {
        return lastAimActive;
    }

    /**
     * Last bearing sample observed while aiming, if any.
     */
    public BearingSample getLastBearingSample() {
        return lastBearingSample;
    }

    /**
     * Last omega command produced by the {@link TagAimController}.
     */
    public double getLastOmegaFromController() {
        return lastOmegaFromController;
    }

    /**
     * Last base drive signal used before any aiming override.
     */
    public DriveSignal getLastBaseSignal() {
        return lastBaseSignal;
    }

    /**
     * Last output signal produced by this drive source (after aiming logic).
     */
    public DriveSignal getLastOutputSignal() {
        return lastOutputSignal;
    }
}
