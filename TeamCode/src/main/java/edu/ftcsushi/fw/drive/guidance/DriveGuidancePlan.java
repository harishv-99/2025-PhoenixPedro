package edu.ftcsushi.fw.drive.guidance;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;

/**
 * Immutable <b>executable</b> configuration for DriveGuidance.
 *
 * <p>DriveGuidance is intentionally split into two concepts:</p>
 * <ul>
 *   <li>{@link DriveGuidanceSpec}: <b>what</b> you want (targets, control frames, feedback policy).</li>
 *   <li>{@link DriveGuidancePlan}: the spec + <b>controller tuning</b> (kP, caps, deadbands).</li>
 * </ul>
 *
 * <p>A plan does not directly move hardware. Instead you convert it into a {@link DriveOverlay} via
 * {@link #overlay()} and apply it to a base {@link edu.ftcsushi.fw.drive.DriveSource} using
 * {@link edu.ftcsushi.fw.drive.DriveSource#overlayWhen}.</p>
 */
public final class DriveGuidancePlan {

    // ------------------------------------------------------------------------
    // Tuning
    // ------------------------------------------------------------------------

    /**
     * Tuning knobs for DriveGuidance controllers.
     *
     * <p><b>Student-friendly explanation:</b> DriveGuidance measures an error (how far you are from
     * a target point, or how many radians you are turned away) and produces a drive command that
     * reduces that error. These constants decide how “strong” the corrections are.</p>
     *
     * <p>The gains convert physical error into normalized
     * {@link edu.ftcsushi.fw.drive.DriveSignal} commands: translation gain is command per inch,
     * and aim gain is omega command per radian. Command caps and the minimum omega command are
     * normalized magnitudes in {@code [0, 1]}; the aim deadband is in radians. These software
     * domains are validated whenever a tuning value is created. They do not prove that the tuning
     * is physically safe or effective for a particular drivetrain.</p>
     *
     * <h2>How to tune (quick checklist)</h2>
     * <ol>
     *   <li>Start with {@link #defaults()}.</li>
     *   <li>If the assist feels <b>too weak</b>: increase the relevant kP.</li>
     *   <li>If it feels <b>twitchy / oscillates</b>: decrease kP, or increase {@link #aimDeadbandRad}.</li>
     *   <li>If it moves/turns <b>too fast</b>: lower {@link #maxTranslateCmd} / {@link #maxOmegaCmd}.</li>
     * </ol>
     *
     * <h2>Typical values</h2>
     * <ul>
     *   <li>{@code maxTranslateCmd}: 0.3–0.8</li>
     *   <li>{@code maxOmegaCmd}: 0.3–1.0</li>
     *   <li>{@code minOmegaCmd}: 0.00–0.10 (0 disables; helps overcome drivetrain stiction)</li>
     *   <li>{@code aimDeadbandRad}: 0.5°–2° (convert to radians with {@code Math.toRadians(...)})</li>
     * </ul>
     */
    public static final class Tuning {

        /**
         * P gain for translation, in normalized translation command per inch of error.
         * Must be finite and non-negative.
         */
        public final double kPTranslate;

        /**
         * Maximum normalized translation command magnitude, in {@code [0, 1]}.
         */
        public final double maxTranslateCmd;

        /**
         * P gain for aim, in normalized omega command per radian of bearing error.
         * Must be finite and non-negative.
         */
        public final double kPAim;

        /**
         * Maximum normalized omega command magnitude, in {@code [0, 1]}.
         */
        public final double maxOmegaCmd;

        /**
         * Minimum normalized omega command magnitude when outside the aim deadband.
         *
         * <p>This is a small "stiction bust" term: many drivetrains won't physically move
         * for very small turn commands due to static friction and motor deadband. When
         * {@code minOmegaCmd > 0}, the controller will output at least this magnitude
         * (with the correct sign) any time the aim error is outside {@link #aimDeadbandRad}.
         *
         * <p>The value must be finite and in {@code [0, maxOmegaCmd]}. A positive value also
         * requires a positive {@link #kPAim}. Typical values are 0.03–0.10; leave it at zero to
         * disable the minimum.</p>
         */
        public final double minOmegaCmd;

        /**
         * Deadband for aim, in radians. Errors at or inside this magnitude output zero omega.
         * Must be finite and in {@code [0, Math.PI]}.
         */
        public final double aimDeadbandRad;

        /**
         * Creates a full tuning bundle.
         *
         * @param kPTranslate      finite, non-negative translation proportional gain in normalized
         *                          command per inch
         * @param maxTranslateCmd finite maximum translation command magnitude in {@code [0, 1]}
         * @param kPAim            finite, non-negative aim proportional gain in normalized omega
         *                          command per radian
         * @param maxOmegaCmd      finite maximum omega command magnitude in {@code [0, 1]}
         * @param minOmegaCmd      finite minimum omega magnitude in {@code [0, maxOmegaCmd]}; a
         *                          positive minimum requires a positive {@code kPAim}
         * @param aimDeadbandRad   finite aim deadband in {@code [0, Math.PI]} radians
         * @throws IllegalArgumentException if a value is non-finite or outside its documented
         *                                  domain, or if the complete tuple is contradictory
         */
        Tuning(double kPTranslate,
               double maxTranslateCmd,
               double kPAim,
               double maxOmegaCmd,
               double minOmegaCmd,
               double aimDeadbandRad) {
            requireFiniteNonNegative("kPTranslate", kPTranslate);
            requireNormalizedMagnitude("maxTranslateCmd", maxTranslateCmd);
            requireFiniteNonNegative("kPAim", kPAim);
            requireNormalizedMagnitude("maxOmegaCmd", maxOmegaCmd);
            requireFiniteNonNegative("minOmegaCmd", minOmegaCmd);
            if (minOmegaCmd > maxOmegaCmd) {
                throw new IllegalArgumentException(
                        "DriveGuidancePlan.Tuning.minOmegaCmd/maxOmegaCmd must satisfy "
                                + "minOmegaCmd <= maxOmegaCmd; received minOmegaCmd="
                                + minOmegaCmd + ", maxOmegaCmd=" + maxOmegaCmd);
            }
            if (minOmegaCmd > 0.0 && kPAim == 0.0) {
                throw new IllegalArgumentException(
                        "DriveGuidancePlan.Tuning.minOmegaCmd/kPAim requires kPAim > 0 when "
                                + "minOmegaCmd > 0; received minOmegaCmd=" + minOmegaCmd
                                + ", kPAim=" + kPAim);
            }
            if (!Double.isFinite(aimDeadbandRad)
                    || aimDeadbandRad < 0.0
                    || aimDeadbandRad > Math.PI) {
                throw new IllegalArgumentException(
                        "DriveGuidancePlan.Tuning.aimDeadbandRad must be finite and in "
                                + "[0, Math.PI]; received " + aimDeadbandRad);
            }
            this.kPTranslate = kPTranslate;
            this.maxTranslateCmd = maxTranslateCmd;
            this.kPAim = kPAim;
            this.maxOmegaCmd = maxOmegaCmd;
            this.minOmegaCmd = minOmegaCmd;
            this.aimDeadbandRad = aimDeadbandRad;
        }

        private static void requireFiniteNonNegative(String field, double value) {
            if (!Double.isFinite(value) || value < 0.0) {
                throw new IllegalArgumentException(
                        "DriveGuidancePlan.Tuning." + field
                                + " must be finite and >= 0; received " + value);
            }
        }

        private static void requireNormalizedMagnitude(String field, double value) {
            if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
                throw new IllegalArgumentException(
                        "DriveGuidancePlan.Tuning." + field
                                + " must be finite and in [0, 1]; received " + value);
            }
        }

        /**
         * Return a copy of this tuning with a different translation kP gain.
         *
         * <p>Higher values make the robot drive toward the translation target more aggressively.</p>
         *
         * @param kPTranslate finite, non-negative normalized command per inch
         * @return an independent validated tuning value
         * @throws IllegalArgumentException if {@code kPTranslate} is non-finite or negative
         */
        public Tuning withTranslateKp(double kPTranslate) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different maximum translation command.
         *
         * <p>This caps how strongly the assist can command X/Y translation.</p>
         *
         * @param maxTranslateCmd finite normalized magnitude in {@code [0, 1]}
         * @return an independent validated tuning value
         * @throws IllegalArgumentException if {@code maxTranslateCmd} is outside its domain
         */
        public Tuning withMaxTranslateCmd(double maxTranslateCmd) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different aim (turn) kP gain.
         *
         * <p>Higher values make the robot turn toward the aim target more aggressively. A zero
         * value is incompatible with a positive {@link #minOmegaCmd}.</p>
         *
         * @param kPAim finite, non-negative normalized omega command per radian
         * @return an independent validated tuning value
         * @throws IllegalArgumentException if {@code kPAim} is outside its domain or contradicts
         *                                  the retained minimum omega command
         */
        public Tuning withAimKp(double kPAim) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different maximum omega (turn) command.
         *
         * <p>This caps how strongly the assist can command a turn.</p>
         *
         * @param maxOmegaCmd finite normalized magnitude in {@code [0, 1]} and not less than the
         *                    retained {@link #minOmegaCmd}
         * @return an independent validated tuning value
         * @throws IllegalArgumentException if {@code maxOmegaCmd} is outside its domain or below
         *                                  the retained minimum omega command
         */
        public Tuning withMaxOmegaCmd(double maxOmegaCmd) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different minimum omega (turn) command.
         *
         * <p>When {@code minOmegaCmd > 0}, DriveGuidance will output at least this magnitude
         * any time the aim error is outside the aim deadband. This helps overcome static
         * friction so the robot doesn't "give up" while still slightly mis-aimed.</p>
         *
         * @param minOmegaCmd finite normalized magnitude in {@code [0, maxOmegaCmd]}; a positive
         *                    value requires a positive retained {@link #kPAim}
         * @return an independent validated tuning value
         * @throws IllegalArgumentException if {@code minOmegaCmd} is outside its domain or
         *                                  contradicts the retained aim gain
         */
        public Tuning withMinOmegaCmd(double minOmegaCmd) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different aim deadband in radians.
         *
         * <p>If the aim error magnitude is at or inside this deadband, DriveGuidance outputs zero
         * turn command.</p>
         *
         * @param aimDeadbandRad finite deadband in {@code [0, Math.PI]} radians
         * @return an independent validated tuning value
         * @throws IllegalArgumentException if {@code aimDeadbandRad} is outside its domain
         */
        public Tuning withAimDeadbandRad(double aimDeadbandRad) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }


        /**
         * Reasonable defaults for typical TeleOp assist.
         */
        public static Tuning defaults() {
            return new Tuning(
                    0.05,                 // kPTranslate
                    0.60,                 // maxTranslateCmd
                    2.50,                 // kPAim
                    0.80,                 // maxOmegaCmd
                    0.00,                 // minOmegaCmd
                    Math.toRadians(1.0)   // aimDeadbandRad
            );
        }
    }

    // ------------------------------------------------------------------------
    // Plan data
    // ------------------------------------------------------------------------

    public final DriveGuidanceSpec spec;
    public final Tuning tuning;

    DriveGuidancePlan(DriveGuidanceSpec spec, Tuning tuning) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.tuning = Objects.requireNonNull(tuning, "tuning");
    }

    /**
     * @return the natural (requested) mask implied by the configured spec targets.
     */
    public DriveOverlayMask requestedMask() {
        return spec.requestedMask();
    }

    /**
     * Create a {@link DriveOverlay} implementing this plan.
     */
    public DriveOverlay overlay() {
        return new DriveGuidanceOverlay(this);
    }

    /**
     * Build a {@link DriveGuidanceTask} that executes this plan as an autonomous-style task.
     *
     * <p>This is the task counterpart to {@link #overlay()}: the same plan and tuning are reused,
     * but the output is sent directly to a {@link DriveCommandSink} until the task reaches its
     * tolerance or timeout. The Task validates and snapshots {@code cfg} during this call; later
     * mutations affect only subsequently created Tasks. A {@code null} config selects
     * {@link DriveGuidanceTask.Config} defaults.</p>
     *
     * <p>Ordinary managed Auto usage:</p>
     * <pre>{@code
     * DriveGuidancePlan plan = DriveGuidance.plan()
     *         .translateTo().fieldPointInches(48.0, 24.0)
     *         .solveWith().localizationOnlyWithDefaults(poseEstimator)
     *         .driveTuning().use(DriveGuidancePlan.Tuning.defaults()).doneDriveTuning()
     *         .build();
     *
     * program.rootTask(plan.task(drivebase, new DriveGuidanceTask.Config()));
     * }</pre>
     *
     * @param drivebase final drive-command owner for this Task
     * @param cfg task tolerances, timeouts, and optional requested-mask override; may be null
     * @return a fresh single-use autonomous guidance Task
     * @throws NullPointerException if {@code drivebase} is null
     * @throws IllegalArgumentException if a numeric config value is outside its documented domain
     */
    public DriveGuidanceTask task(DriveCommandSink drivebase, DriveGuidanceTask.Config cfg) {
        return new DriveGuidanceTask(drivebase, this, cfg);
    }

    /**
     * Build a {@link DriveGuidanceQuery} that can sample this plan's errors (and predicted drive command)
     * without enabling an overlay.
     */
    public DriveGuidanceQuery query() {
        return new DriveGuidanceQuery(this);
    }
}
