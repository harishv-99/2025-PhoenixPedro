package edu.ftcphoenix.fw.sensing;

import java.util.Objects;

import edu.ftcphoenix.fw.core.PidController;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.source.TagAimDriveSource;
import edu.ftcphoenix.fw.input.Button;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.WaitUntilTask;

/**
 * Helpers for tag-based auto-aim that wrap an existing {@link DriveSource}.
 *
 * <h2>Role</h2>
 * <p>{@code TagAim} is a convenience facade that wires together:</p>
 *
 * <ul>
 *   <li>A source of target bearing (typically built from a {@link TagTarget} or
 *       directly from an {@link AprilTagSensor} via a {@link BearingSource}).</li>
 *   <li>A {@link TagAimController} that turns bearing error into an omega command.</li>
 *   <li>A {@link TagAimDriveSource} that overrides only the turn (omega) portion
 *       of an existing {@link DriveSource} while an aim button is held.</li>
 * </ul>
 *
 * <p>Typical TeleOp usage with {@link TagTarget}:</p>
 *
 * <pre>{@code
 * // In init():
 * AprilTagSensor tagSensor = FtcVision.aprilTags(hardwareMap, "Webcam 1");
 * Set<Integer> scoringTags = Set.of(1, 2, 3);
 *
 * // Track the "best" scoring tag each loop.
 * TagTarget scoringTarget = new TagTarget(tagSensor, scoringTags, 0.5);
 *
 * // Map driver sticks to mecanum drive.
 * DriveSource sticks = GamepadDriveSource.teleOpMecanumStandard(gamepads);
 *
 * // Wrap with TagAim: hold left bumper to auto-aim toward the tracked tag.
 * DriveSource drive = TagAim.teleOpAim(
 *         sticks,
 *         gamepads.p1().leftBumper(),
 *         scoringTarget
 * );
 *
 * // In your loop():
 * scoringTarget.update();                 // once per loop
 * DriveSignal cmd = drive.get(clock);     // TagAimDriveSource handles omega
 * drivebase.drive(cmd);
 * drivebase.update(clock);
 * }</pre>
 *
 * <p>Under the hood it:</p>
 * <ul>
 *   <li>Reads the current bearing from a {@link TagTarget} (or generic {@link BearingSource}).</li>
 *   <li>Uses a {@link TagAimController} to turn bearing into omega.</li>
 *   <li>Preserves the original {@link DriveSource} so that when the aim
 *       button is held, {@code omega} is overridden to aim at the tag,
 *       but axial/lateral still come from the driver.</li>
 * </ul>
 *
 * <h2>Usage levels</h2>
 * <ul>
 *   <li><b>Beginner:</b> create a {@link TagTarget} and call
 *       {@link #teleOpAim(DriveSource, Button, TagTarget)}.</li>
 *   <li><b>Tunable:</b> provide a {@link Config} to
 *       {@link #teleOpAim(DriveSource, Button, TagTarget, Config)} to adjust
 *       gain, deadband, maximum omega, or lost-target behavior.</li>
 *   <li><b>Advanced:</b> build your own {@link BearingSource} and
 *       {@link TagAimController}, then call
 *       {@link #teleOpAim(DriveSource, Button, BearingSource, TagAimController)}.
 *       You can also use {@link #controllerFromConfig(Config)} directly to
 *       build a controller for autonomous code.</li>
 * </ul>
 */
public final class TagAim {

    /**
     * Default proportional gain for simple P-only aiming.
     */
    private static final double DEFAULT_KP = 4.0;

    /**
     * Default deadband for aiming, in radians.
     *
     * <p>This corresponds to roughly 1 degree.</p>
     */
    private static final double DEFAULT_DEADBAND_RAD = Math.toRadians(1.0);

    /**
     * Default maximum turn rate command for aiming.
     */
    private static final double DEFAULT_MAX_OMEGA = 0.8;

    private TagAim() {
        // Utility class; no instances.
    }

    // ------------------------------------------------------------------------
    // Generic configuration
    // ------------------------------------------------------------------------

    /**
     * Generic configuration for tag-based aiming.
     *
     * <p>
     * This configuration is not TeleOp-specific. The same object can be used
     * to:
     * </p>
     * <ul>
     *   <li>Tune the built-in TeleOp helper
     *       ({@link #teleOpAim(DriveSource, Button, TagTarget, Config)}), or</li>
     *   <li>Build a {@link TagAimController} directly for autonomous logic via
     *       {@link #controllerFromConfig(Config)}.</li>
     * </ul>
     */
    public static final class Config {

        /**
         * Proportional gain for the simple P-only controller
         * ({@code omega = kp * bearingError}).
         *
         * <p>Default: {@value TagAim#DEFAULT_KP}.</p>
         */
        public double kp = DEFAULT_KP;

        /**
         * Deadband for aiming, in radians.
         *
         * <p>
         * When {@code |bearing| < deadbandRad}, the controller outputs zero
         * omega and does not update the PID state.
         * </p>
         *
         * <p>Default: {@link TagAim#DEFAULT_DEADBAND_RAD}.</p>
         */
        public double deadbandRad = DEFAULT_DEADBAND_RAD;

        /**
         * Absolute maximum turn-rate command (omega).
         *
         * <p>Default: {@value TagAim#DEFAULT_MAX_OMEGA}.</p>
         */
        public double maxOmega = DEFAULT_MAX_OMEGA;

        /**
         * Policy used when the target is lost.
         *
         * <p>Default: {@link TagAimController.LossPolicy#ZERO_OUTPUT_RESET_I}.</p>
         */
        public TagAimController.LossPolicy lossPolicy =
                TagAimController.LossPolicy.ZERO_OUTPUT_RESET_I;

        private Config() {
            // Use defaults().
        }

        /**
         * Create a new configuration instance with default values.
         *
         * @return a new {@link Config} instance
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Create a shallow copy of this configuration.
         *
         * <p>
         * Useful when you want to start from a base config, tweak a few fields,
         * and keep the original unchanged.
         * </p>
         *
         * @return a new {@link Config} with the same field values
         */
        public Config copy() {
            Config c = new Config();
            c.kp = this.kp;
            c.deadbandRad = this.deadbandRad;
            c.maxOmega = this.maxOmega;
            c.lossPolicy = this.lossPolicy;
            return c;
        }
    }

    // ------------------------------------------------------------------------
    // Beginner / tunable TeleOp API: TagTarget
    // ------------------------------------------------------------------------

    /**
     * TeleOp helper: wrap an existing drive source with tag-based auto-aim,
     * using a {@link TagTarget}.
     *
     * <p>This overload is intended for most teams. It:</p>
     *
     * <ul>
     *   <li>Uses {@link TagTarget#toBearingSample()} to read the current bearing.</li>
     *   <li>Uses a reasonable default {@link TagAimController} built from
     *       {@link Config#defaults()}.</li>
     *   <li>Returns a {@link DriveSource} that:
     *     <ul>
     *       <li>Forwards the base drive command when {@code aimButton} is not pressed.</li>
     *       <li>Overrides only omega while {@code aimButton} is pressed.</li>
     *     </ul>
     *   </li>
     * </ul>
     *
     * @param baseDrive existing drive source (e.g., stick mapping)
     * @param aimButton button that enables aiming while pressed
     * @param target    tag target providing the current observation
     * @return a new {@link DriveSource} that adds aiming behavior on top of {@code baseDrive}
     */
    public static DriveSource teleOpAim(
            DriveSource baseDrive,
            Button aimButton,
            TagTarget target) {

        return teleOpAim(baseDrive, aimButton, target, Config.defaults());
    }

    /**
     * TeleOp helper: same as
     * {@link #teleOpAim(DriveSource, Button, TagTarget)} but with an explicit
     * {@link Config} to override the defaults.
     *
     * <p>
     * This is useful when you want the convenience of the beginner API
     * (automatic {@link TagTarget} wiring) but need to tune parameters such as
     * proportional gain, deadband, maximum omega, or lost-target behavior.
     * </p>
     *
     * @param baseDrive existing drive source (e.g., stick mapping)
     * @param aimButton button that enables aiming while pressed
     * @param target    tag target providing the current observation; must not be null
     * @param config    configuration for aiming; if {@code null},
     *                  {@link Config#defaults()} is used
     * @return a new {@link DriveSource} that adds aiming behavior on top of {@code baseDrive}
     */
    public static DriveSource teleOpAim(
            DriveSource baseDrive,
            Button aimButton,
            TagTarget target,
            Config config) {

        if (target == null) {
            throw new IllegalArgumentException("target must not be null");
        }

        Config cfg = (config != null) ? config : Config.defaults();

        BearingSource bearing = clock -> target.toBearingSample();
        TagAimController controller = controllerFromConfig(cfg);

        return teleOpAim(baseDrive, aimButton, bearing, controller);
    }

    // ------------------------------------------------------------------------
    // Aim readiness helpers: tasks that wait for alignment
    // ------------------------------------------------------------------------

    /**
     * Create a {@link Task} that waits until the given {@link TagTarget}'s
     * bearing is within a specified angular tolerance.
     *
     * <p>This is a thin convenience wrapper around
     * {@link TagTarget#isBearingWithin(double)} and {@link WaitUntilTask} that
     * lets you express "wait until aim is good" as a reusable task. It
     * performs no drive control by itself; it only watches the target state.
     * </p>
     *
     * <p>Semantics:</p>
     * <ul>
     *   <li>If {@link TagTarget#hasTarget()} is {@code false}, the condition is
     *       treated as not satisfied.</li>
     *   <li>The task completes successfully when
     *       {@code target.isBearingWithin(toleranceRad)} first returns
     *       {@code true}.</li>
     *   <li>There is no timeout; if the condition never becomes true, this task
     *       can run indefinitely. If you want a timeout, use the overload that
     *       accepts {@code timeoutSec}.</li>
     * </ul>
     *
     * @param target       tag target to observe; must not be {@code null}
     * @param toleranceRad non-negative angular tolerance in radians
     * @return a {@link Task} that reports {@link edu.ftcphoenix.fw.task.TaskOutcome#SUCCESS}
     *         when the aim is within tolerance
     * @throws IllegalArgumentException if {@code toleranceRad} is negative
     */
    public static Task waitForAim(TagTarget target, double toleranceRad) {
        Objects.requireNonNull(target, "target is required");
        if (toleranceRad < 0.0) {
            throw new IllegalArgumentException("toleranceRad must be >= 0, got " + toleranceRad);
        }

        return new WaitUntilTask(() -> target.isBearingWithin(toleranceRad));
    }

    /**
     * Create a {@link Task} that waits until the given {@link TagTarget}'s
     * bearing is within a specified angular tolerance, but gives up if it
     * takes longer than {@code timeoutSec}.
     *
     * <p>Outcome semantics:</p>
     * <ul>
     *   <li>If the aim enters the tolerance band before the timeout, the task
     *       completes with {@link edu.ftcphoenix.fw.task.TaskOutcome#SUCCESS}.</li>
     *   <li>If {@code timeoutSec} elapses first, the task completes with
     *       {@link edu.ftcphoenix.fw.task.TaskOutcome#TIMEOUT}.</li>
     * </ul>
     *
     * <p>Typical usage in a shooter macro:</p>
     * <pre>{@code
     * Task waitForAim = TagAim.waitForAim(scoringTarget,
     *                                     aimConfig.deadbandRad,
     *                                     1.0 /* timeout in seconds *\/);
     * }</pre>
     *
     * @param target       tag target to observe; must not be {@code null}
     * @param toleranceRad non-negative angular tolerance in radians
     * @param timeoutSec   timeout in seconds; must be {@code >= 0.0}.
     *                     A value of {@code 0.0} means "fail immediately unless
     *                     we are already within tolerance".
     * @return a {@link Task} that reports {@link edu.ftcphoenix.fw.task.TaskOutcome#SUCCESS}
     *         when the aim is within tolerance, or
     *         {@link edu.ftcphoenix.fw.task.TaskOutcome#TIMEOUT} if the timeout elapses
     * @throws IllegalArgumentException if {@code toleranceRad} or {@code timeoutSec} is negative
     */
    public static Task waitForAim(TagTarget target,
                                  double toleranceRad,
                                  double timeoutSec) {
        Objects.requireNonNull(target, "target is required");
        if (toleranceRad < 0.0) {
            throw new IllegalArgumentException("toleranceRad must be >= 0, got " + toleranceRad);
        }
        if (timeoutSec < 0.0) {
            throw new IllegalArgumentException("timeoutSec must be >= 0, got " + timeoutSec);
        }

        return new WaitUntilTask(
                () -> target.isBearingWithin(toleranceRad),
                timeoutSec
        );
    }

    // ------------------------------------------------------------------------
    // Advanced API: generic bearing source + controller
    // ------------------------------------------------------------------------

    /**
     * TeleOp helper: wrap an existing drive source with generic bearing-based
     * auto-aim.
     *
     * <p>This overload lets advanced users customize:</p>
     * <ul>
     *   <li>Where bearing information comes from ({@link BearingSource}).</li>
     *   <li>How bearing is converted into omega ({@link TagAimController}).</li>
     * </ul>
     *
     * <p>Behavior:</p>
     * <ul>
     *   <li>On each call to {@link DriveSource#get(edu.ftcphoenix.fw.util.LoopClock)}:
     *     <ul>
     *       <li>Compute the base command from {@code baseDrive}.</li>
     *       <li>If {@code aimButton} is not pressed, return base unchanged.</li>
     *       <li>Otherwise:
     *         <ul>
     *           <li>Sample bearing via
     *               {@link BearingSource#sample(edu.ftcphoenix.fw.util.LoopClock)}.</li>
     *           <li>Call
     *               {@link TagAimController#update(edu.ftcphoenix.fw.util.LoopClock,
     *               BearingSource.BearingSample)}
     *               to get omega.</li>
     *           <li>Return a new command that keeps base axial/lateral but uses
     *               the aiming omega.</li>
     *         </ul>
     *       </li>
     *     </ul>
     *   </li>
     * </ul>
     *
     * @param baseDrive  existing drive source (e.g., sticks, planner)
     * @param aimButton  button that enables aiming while pressed
     * @param bearing    bearing source (target angle) used for aiming
     * @param controller controller that turns bearing into omega
     * @return a {@link DriveSource} that adds aiming behavior on top of {@code baseDrive}
     */
    public static DriveSource teleOpAim(
            DriveSource baseDrive,
            Button aimButton,
            BearingSource bearing,
            TagAimController controller) {

        // Delegate to the reusable TagAimDriveSource so we don't duplicate logic
        // and we get debugDump() support for free.
        return new TagAimDriveSource(baseDrive, aimButton, bearing, controller);
    }

    // ------------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------------

    /**
     * Build a {@link TagAimController} from a {@link Config}.
     *
     * <p>
     * This is a general-purpose helper: it is used by the TeleOp helpers, and
     * can also be used directly in autonomous code that wants to control
     * {@code omega} from tag bearing without going through
     * {@link TagAimDriveSource}.
     * </p>
     *
     * @param cfg configuration; if {@code null}, {@link Config#defaults()} is used
     * @return a new {@link TagAimController} instance
     */
    public static TagAimController controllerFromConfig(Config cfg) {
        if (cfg == null) {
            cfg = Config.defaults();
        }

        final double kp = cfg.kp;

        PidController pid = new PidController() {
            @Override
            public double update(double error, double dtSec) {
                // P-only: omega = Kp * error
                return kp * error;
            }
            // Default reset() is fine (no internal state).
        };

        double deadbandRad = cfg.deadbandRad;
        TagAimController.LossPolicy loss =
                (cfg.lossPolicy != null)
                        ? cfg.lossPolicy
                        : TagAimController.LossPolicy.ZERO_OUTPUT_RESET_I;

        return new TagAimController(
                pid,
                deadbandRad,
                cfg.maxOmega,
                loss
        );
    }
}
