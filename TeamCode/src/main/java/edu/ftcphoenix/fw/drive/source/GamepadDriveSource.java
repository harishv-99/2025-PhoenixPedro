package edu.ftcphoenix.fw.drive.source;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.localization.HeadingEstimate;
import edu.ftcphoenix.fw.localization.HeadingEstimator;

/**
 * {@link DriveSource} that maps explicit gamepad-style axes to a robot-centric {@link DriveSignal}
 * for TeleOp driving (for example, mecanum).
 *
 * <h2>What this class is responsible for</h2>
 * <ul>
 *   <li>Mapping <em>explicitly supplied axes</em> to drive intent (axial / lateral / omega).</li>
 *   <li>Stick shaping (deadband + exponent) and scaling (max translate / max omega).</li>
 *   <li>Nothing about button choices, driver slots, or slow-mode policy.</li>
 * </ul>
 *
 * <p>
 * That last point is intentional. The framework treats button bindings and operator semantics as
 * robot-owned policy, not as part of this primitive. A future robot may use different gamepads,
 * different axes, split-driver control, trigger-based turning, or non-gamepad sources entirely.
 * {@code GamepadDriveSource} should still be reusable in all of those cases.
 * </p>
 *
 * <h2>Phoenix sign conventions</h2>
 * <p>{@link DriveSignal} uses Phoenix conventions:</p>
 * <ul>
 *   <li>{@code axial > 0}   → forward</li>
 *   <li>{@code lateral > 0} → left</li>
 *   <li>{@code omega > 0}   → CCW (turn left)</li>
 * </ul>
 *
 * <p>
 * Standard FTC stick intuition is typically “stick right means right / clockwise”. This class
 * preserves that driver intuition by converting signs at the boundary:
 * </p>
 * <ul>
 *   <li>lateral raw +right becomes {@code lateral < 0} (right strafe) → inverted</li>
 *   <li>omega raw +clockwise becomes {@code omega < 0} (clockwise) → inverted</li>
 * </ul>
 *
 * <h2>Recommended usage</h2>
 *
 * <p>The {@code GamepadDevice} below is the FTC-boundary
 * {@link edu.ftcphoenix.fw.ftc.input.GamepadDevice GamepadDevice} adapter. This source itself
 * receives only Phoenix scalar and boolean sources.</p>
 *
 * <pre>{@code
 * GamepadDevice driver = gamepads.p1();
 * GamepadDriveSource.Config cfg = GamepadDriveSource.Config.defaults();
 *
 * DriveSource manual = new GamepadDriveSource(
 *         driver.leftX(),
 *         driver.leftY(),
 *         driver.rightX(),
 *         cfg
 * ).scaledWhen(driver.rightBumper(), 0.35, 0.20);
 * }</pre>
 *
 * <p>
 * The important part is that the robot code explicitly chooses the axes and any slow-mode button.
 * This class only performs axis-to-command mapping.
 * </p>
 *
 * <h2>Note on shaping</h2>
 * <p>
 * Stick shaping uses {@link ScalarSource#shaped(double, double, double, double)} with min/max of
 * {@code [-1, +1]} because this class is mapping normalized controller-style axes. Each raw
 * axis is sampled once per {@link #get(LoopClock)} call; that one sample drives both the command
 * and its diagnostics.
 * </p>
 */
public final class GamepadDriveSource implements DriveSource {

    private static final double DEFAULT_MAX_HEADING_AGE_SEC = 0.25;
    private static final double DEFAULT_MIN_HEADING_QUALITY = 0.0;

    /**
     * Configuration for TeleOp stick shaping.
     *
     * <p>
     * This is a mutable data object. {@link GamepadDriveSource} makes and validates a defensive
     * copy when it is constructed. {@link #copy()} itself remains a raw data copy so callers may
     * edit a draft before handing it to the source that owns these rules.
     * </p>
     */
    public static final class Config {

        /**
         * Finite symmetric deadband radius in [0, 1]. Default: 0.05.
         *
         * <p>
         * Values with {@code |v| <= deadband} are treated as 0. Values outside the deadband are
         * normalized before the exponent is applied.
         * </p>
         */
        public double deadband = 0.05;

        /**
         * Finite exponent greater than or equal to 1 for translation (axial + lateral).
         * Default: 1.5.
         *
         * <p>Values &gt; 1 soften near center and keep full-scale at the edges.</p>
         */
        public double translateExpo = 1.5;

        /**
         * Finite exponent greater than or equal to 1 for rotation (omega). Default: 1.5.
         */
        public double rotateExpo = 1.5;

        /**
         * Finite max translation scale in [0, 1] applied after shaping. Default: 1.0.
         */
        public double translateScale = 1.0;

        /**
         * Finite max rotation scale in [0, 1] applied after shaping. Default: 1.0.
         */
        public double rotateScale = 1.0;

        private Config() {
            // Defaults set via field initializers.
        }

        /**
         * Creates a config populated with Phoenix defaults.
         *
         * @return new mutable config initialized to the framework defaults
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Creates a deep copy of this config.
         *
         * @return copied config whose fields can be edited independently
         */
        public Config copy() {
            Config c = new Config();
            c.deadband = this.deadband;
            c.translateExpo = this.translateExpo;
            c.rotateExpo = this.rotateExpo;
            c.translateScale = this.translateScale;
            c.rotateScale = this.rotateScale;
            return c;
        }
    }

    // Raw axes (sampled for debug).
    private final ScalarSource axisLateralRaw; // raw +right (typical)
    private final ScalarSource axisAxialRaw;   // +forward in robot-driving intuition
    private final ScalarSource axisOmegaRaw;   // raw +clockwise (typical)

    // Shaped wrappers read the one per-get snapshot retained in the diagnostic fields below.
    private final ScalarSource axisAxialCmd;
    private final ScalarSource axisLateralCmd; // +left
    private final ScalarSource axisOmegaCmd;   // +CCW

    private final Config cfg;

    private DriveSignal lastSignal = DriveSignal.zero();

    // Cached raw axis samples from the most recent get(clock) call (for debug).
    private double lastLateralRaw = 0.0;
    private double lastAxialRaw = 0.0;
    private double lastOmegaRaw = 0.0;

    /**
     * Core constructor: map three raw axes into a drive signal using {@link Config}.
     *
     * @param axisLateralRaw raw lateral axis (typically +right)
     * @param axisAxialRaw axial axis (typically +forward)
     * @param axisOmegaRaw raw omega axis (typically +clockwise / turn-right)
     * @param cfg stick-shaping configuration; defensively copied and validated
     * @throws IllegalArgumentException if an axis or {@code cfg} is null, if the deadband or a
     *         scale is non-finite or outside [0, 1], or if an exponent is non-finite or less than
     *         1
     */
    public GamepadDriveSource(ScalarSource axisLateralRaw,
                              ScalarSource axisAxialRaw,
                              ScalarSource axisOmegaRaw,
                              Config cfg) {
        if (axisLateralRaw == null) {
            throw new IllegalArgumentException("axisLateralRaw is required");
        }
        if (axisAxialRaw == null) {
            throw new IllegalArgumentException("axisAxialRaw is required");
        }
        if (axisOmegaRaw == null) {
            throw new IllegalArgumentException("axisOmegaRaw is required");
        }
        if (cfg == null) {
            throw new IllegalArgumentException("GamepadDriveSource.Config is required");
        }

        Config snapshot = cfg.copy();
        validateConfig(snapshot);

        this.axisLateralRaw = axisLateralRaw;
        this.axisAxialRaw = axisAxialRaw;
        this.axisOmegaRaw = axisOmegaRaw;
        this.cfg = snapshot;

        // Keep shaping in ScalarSource while decoupling it from the externally supplied sources.
        // get(clock) captures each external axis once, then these wrappers all read that snapshot.
        this.axisAxialCmd = ScalarSource.of(() -> lastAxialRaw)
                .shaped(this.cfg.deadband, this.cfg.translateExpo, -1.0, 1.0)
                .scaled(this.cfg.translateScale);
        this.axisLateralCmd = ScalarSource.of(() -> lastLateralRaw)
                .shaped(this.cfg.deadband, this.cfg.translateExpo, -1.0, 1.0)
                .scaled(this.cfg.translateScale)
                .inverted();
        this.axisOmegaCmd = ScalarSource.of(() -> lastOmegaRaw)
                .shaped(this.cfg.deadband, this.cfg.rotateExpo, -1.0, 1.0)
                .scaled(this.cfg.rotateScale)
                .inverted();
    }

    private static void validateConfig(Config config) {
        requireFiniteUnitInterval("deadband", config.deadband);
        requireFiniteAtLeastOne("translateExpo", config.translateExpo);
        requireFiniteAtLeastOne("rotateExpo", config.rotateExpo);
        requireFiniteUnitInterval("translateScale", config.translateScale);
        requireFiniteUnitInterval("rotateScale", config.rotateScale);
    }

    private static void requireFiniteUnitInterval(String field, double value) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    "GamepadDriveSource.Config." + field
                            + " must be finite and in [0.0, 1.0], got " + value + "."
            );
        }
    }

    private static void requireFiniteAtLeastOne(String field, double value) {
        if (!Double.isFinite(value) || value < 1.0) {
            throw new IllegalArgumentException(
                    "GamepadDriveSource.Config." + field
                            + " must be finite and >= 1.0, got " + value + "."
            );
        }
    }

    /**
     * Samples the explicit axes and returns the current robot-centric drive command.
     *
     * @param clock shared loop clock used to sample the underlying sources
     * @return current robot-centric drive signal after shaping, scaling, and sign conversion
     */
    @Override
    public DriveSignal get(LoopClock clock) {
        // These are the calibrated (but unshaped) upstream values. Keep them local until all three
        // samples succeed so command and diagnostics describe the same sampling pass.
        double lateralRaw = axisLateralRaw.getAsDouble(clock);
        double axialRaw = axisAxialRaw.getAsDouble(clock);
        double omegaRaw = axisOmegaRaw.getAsDouble(clock);

        lastLateralRaw = lateralRaw;
        lastAxialRaw = axialRaw;
        lastOmegaRaw = omegaRaw;

        double ax = axisAxialCmd.getAsDouble(clock);
        double lat = axisLateralCmd.getAsDouble(clock);
        double om = axisOmegaCmd.getAsDouble(clock);

        DriveSignal out = new DriveSignal(ax, lat, om);
        lastSignal = out;
        return out;
    }

    /**
     * Interpret this source's shaped translation as control-frame intent and convert it to the
     * robot frame using cached heading evidence.
     *
     * <p>The supplied up heading is authored in the same fixed field frame as the estimator. It is
     * resolved once after construction or reset. The returned source preserves omega and outputs
     * zero translation instead of silently falling back to robot-relative controls when heading
     * evidence is unusable.</p>
     *
     * @param headingEstimator externally updated cached field-heading evidence
     * @param configuredUpHeadingSupplier cache-only finite field heading supplier, normally backed
     *                                    by a START-frozen named station
     * @return robot-centric drive source using default heading-evidence thresholds
     */
    public DriveSource fieldRelativeTo(HeadingEstimator headingEstimator,
                                       DoubleSupplier configuredUpHeadingSupplier) {
        return fieldRelativeTo(
                headingEstimator,
                configuredUpHeadingSupplier,
                DEFAULT_MAX_HEADING_AGE_SEC,
                DEFAULT_MIN_HEADING_QUALITY
        );
    }

    /**
     * Advanced field-relative conversion with explicit heading-evidence thresholds.
     *
     * @param headingEstimator externally updated cached field-heading evidence
     * @param configuredUpHeadingSupplier cache-only finite field heading supplier, sampled once
     * @param maxHeadingAgeSec inclusive finite non-negative maximum evidence age
     * @param minHeadingQuality inclusive finite minimum quality in [0, 1]
     * @return robot-centric drive source with field-relative manual translation
     */
    public DriveSource fieldRelativeTo(HeadingEstimator headingEstimator,
                                       DoubleSupplier configuredUpHeadingSupplier,
                                       double maxHeadingAgeSec,
                                       double minHeadingQuality) {
        HeadingEstimator requiredHeading = Objects.requireNonNull(
                headingEstimator,
                "headingEstimator is required"
        );
        DoubleSupplier requiredUp = Objects.requireNonNull(
                configuredUpHeadingSupplier,
                "configuredUpHeadingSupplier is required"
        );
        if (!Double.isFinite(maxHeadingAgeSec) || maxHeadingAgeSec < 0.0) {
            throw new IllegalArgumentException(
                    "maxHeadingAgeSec must be finite and >= 0, got " + maxHeadingAgeSec);
        }
        if (!Double.isFinite(minHeadingQuality)
                || minHeadingQuality < 0.0
                || minHeadingQuality > 1.0) {
            throw new IllegalArgumentException(
                    "minHeadingQuality must be finite and in [0, 1], got "
                            + minHeadingQuality);
        }

        GamepadDriveSource self = this;
        return new DriveSource() {
            private long lastCycle = Long.MIN_VALUE;
            private long failedCycle = Long.MIN_VALUE;
            private RuntimeException retainedFailure;
            private boolean sampling;
            private boolean upResolved;
            private double upFieldHeadingRad;
            private String rejection = "no sample";
            private DriveSignal lastControl = DriveSignal.zero();
            private DriveSignal lastRobot = DriveSignal.zero();

            @Override
            public DriveSignal get(LoopClock clock) {
                Objects.requireNonNull(clock, "clock is required");
                long cycle = clock.cycle();
                if (cycle == failedCycle) {
                    throw retainedFailure;
                }
                if (sampling) {
                    throw new IllegalStateException(
                            "field-relative GamepadDriveSource sample is reentrant");
                }
                if (cycle == lastCycle) {
                    return lastRobot;
                }
                sampling = true;
                try {
                    DriveSignal control = self.get(clock);
                    HeadingEstimate heading = requiredHeading.getHeadingEstimate();
                    resolveUp();
                    String nextRejection = rejectionOf(heading, clock);
                    DriveSignal robot;
                    if (nextRejection == null) {
                        double theta = MathUtil.wrapToPi(
                                heading.fieldHeadingRad - upFieldHeadingRad
                        );
                        double cos = Math.cos(theta);
                        double sin = Math.sin(theta);
                        robot = new DriveSignal(
                                cos * control.axial + sin * control.lateral,
                                -sin * control.axial + cos * control.lateral,
                                control.omega
                        );
                        rejection = "none";
                    } else {
                        robot = new DriveSignal(0.0, 0.0, control.omega);
                        rejection = nextRejection;
                    }
                    lastControl = control;
                    lastRobot = robot;
                    lastCycle = cycle;
                    failedCycle = Long.MIN_VALUE;
                    retainedFailure = null;
                    return robot;
                } catch (RuntimeException failure) {
                    failedCycle = cycle;
                    retainedFailure = failure;
                    throw failure;
                } finally {
                    sampling = false;
                }
            }

            private void resolveUp() {
                if (upResolved) {
                    return;
                }
                double supplied = requiredUp.getAsDouble();
                if (!Double.isFinite(supplied)) {
                    throw new IllegalStateException(
                            "configured control-up field heading must be finite, got " + supplied);
                }
                upFieldHeadingRad = MathUtil.wrapToPi(supplied);
                upResolved = true;
            }

            private String rejectionOf(HeadingEstimate heading, LoopClock clock) {
                if (heading == null || !heading.hasHeading) {
                    return "no heading";
                }
                if (!Double.isFinite(heading.fieldHeadingRad)
                        || !Double.isFinite(heading.quality)) {
                    return "non-finite heading evidence";
                }
                if (heading.quality < minHeadingQuality) {
                    return "heading quality below minimum";
                }
                try {
                    double ageSec = heading.timestamp.ageSec(clock);
                    if (!Double.isFinite(ageSec)) {
                        return "heading timestamp is invalid for this clock epoch";
                    }
                    if (ageSec > maxHeadingAgeSec) {
                        return "heading evidence is stale";
                    }
                } catch (IllegalArgumentException differentClock) {
                    return "heading timestamp belongs to another clock";
                }
                return null;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                failedCycle = Long.MIN_VALUE;
                retainedFailure = null;
                sampling = false;
                upResolved = false;
                upFieldHeadingRad = 0.0;
                rejection = "no sample";
                lastControl = DriveSignal.zero();
                lastRobot = DriveSignal.zero();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = prefix == null || prefix.isEmpty() ? "fieldRelativeDrive" : prefix;
                dbg.addData(p + ".class", "FieldRelativeGamepadDrive")
                        .addData(p + ".rejection", rejection)
                        .addData(p + ".configuredUpFieldHeadingRad",
                                upResolved ? upFieldHeadingRad : Double.NaN)
                        .addData(p + ".lastControl", lastControl)
                        .addData(p + ".lastRobot", lastRobot);
                requiredHeading.debugDumpHeading(dbg, p + ".heading");
                self.debugDump(dbg, p + ".gamepad");
            }
        };
    }

    /**
     * Clear local diagnostics and reset the three structural axis sources.
     */
    @Override
    public void reset() {
        lastLateralRaw = 0.0;
        lastAxialRaw = 0.0;
        lastOmegaRaw = 0.0;
        lastSignal = DriveSignal.zero();
        axisLateralRaw.reset();
        axisAxialRaw.reset();
        axisOmegaRaw.reset();
    }

    /**
     * Dumps internal state to a {@link DebugSink}.
     *
     * <p>
     * Because {@link ScalarSource} requires a {@link LoopClock} to sample, this method reports the
     * most recent raw values cached during {@link #get(LoopClock)}.
     * </p>
     *
     * @param dbg debug sink to write to; ignored when {@code null}
     * @param prefix key prefix for all entries; may be {@code null} or empty
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "sticks" : prefix;

        dbg.addLine(p + ": GamepadDriveSource");

        dbg.addData(p + ".axis.lateral.raw", lastLateralRaw);
        dbg.addData(p + ".axis.axial.raw", lastAxialRaw);
        dbg.addData(p + ".axis.omega.raw", lastOmegaRaw);

        dbg.addData(p + ".last.axial", lastSignal.axial);
        dbg.addData(p + ".last.lateral", lastSignal.lateral);
        dbg.addData(p + ".last.omega", lastSignal.omega);

        dbg.addData(p + ".cfg.deadband", cfg.deadband);
        dbg.addData(p + ".cfg.translateExpo", cfg.translateExpo);
        dbg.addData(p + ".cfg.rotateExpo", cfg.rotateExpo);
        dbg.addData(p + ".cfg.translateScale", cfg.translateScale);
        dbg.addData(p + ".cfg.rotateScale", cfg.rotateScale);
    }

    /**
     * Returns the most recently computed drive command.
     *
     * @return last command produced by {@link #get(LoopClock)}; initially {@link DriveSignal#zero()}
     */
    public DriveSignal getLastSignal() {
        return lastSignal;
    }
}
