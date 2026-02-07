package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;

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
 * {@link #overlay()} and apply it to a base {@link edu.ftcphoenix.fw.drive.DriveSource} using
 * {@link edu.ftcphoenix.fw.drive.DriveSource#overlayWhen}.</p>
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
     * <p>These values are intentionally <b>unitless</b> and operate directly in
     * {@link edu.ftcphoenix.fw.drive.DriveSignal} units (roughly -1..1). That makes them easy to use
     * in TeleOp without any drivetrain characterization.</p>
     *
     * <h3>How to tune (quick checklist)</h3>
     * <ol>
     *   <li>Start with {@link #defaults()}.</li>
     *   <li>If the assist feels <b>too weak</b>: increase the relevant kP.</li>
     *   <li>If it feels <b>twitchy / oscillates</b>: decrease kP, or increase {@link #aimDeadbandRad}.</li>
     *   <li>If it moves/turns <b>too fast</b>: lower {@link #maxTranslateCmd} / {@link #maxOmegaCmd}.</li>
     * </ol>
     *
     * <h3>Typical values</h3>
     * <ul>
     *   <li>{@code maxTranslateCmd}: 0.3–0.8</li>
     *   <li>{@code maxOmegaCmd}: 0.3–1.0</li>
     *   <li>{@code minOmegaCmd}: 0.00–0.10 (0 disables; helps overcome drivetrain stiction)</li>
     *   <li>{@code aimDeadbandRad}: 0.5°–2° (convert to radians with {@code Math.toRadians(...)})</li>
     * </ul>
     */
    public static final class Tuning {

        /**
         * P gain for translation: command per inch of error.
         */
        public final double kPTranslate;

        /**
         * Max translation command magnitude (0..1-ish).
         */
        public final double maxTranslateCmd;

        /**
         * P gain for aim: omega command per radian of bearing error.
         */
        public final double kPAim;

        /**
         * Max omega command magnitude (0..1-ish).
         */
        public final double maxOmegaCmd;

        /**
         * Minimum omega command magnitude (0..1-ish) when outside the aim deadband.
         *
         * <p>This is a small "stiction bust" term: many drivetrains won't physically move
         * for very small turn commands due to static friction and motor deadband. When
         * {@code minOmegaCmd > 0}, the controller will output at least this magnitude
         * (with the correct sign) any time the aim error is outside {@link #aimDeadbandRad}.
         *
         * <p>Typical values: 0.03–0.10. Leave at 0 to disable.</p>
         */
        public final double minOmegaCmd;

        /**
         * Deadband for aim (radians): errors smaller than this output 0 omega.
         */
        public final double aimDeadbandRad;

        public Tuning(double kPTranslate,
                      double maxTranslateCmd,
                      double kPAim,
                      double maxOmegaCmd,
                      double minOmegaCmd,
                      double aimDeadbandRad) {
            this.kPTranslate = kPTranslate;
            this.maxTranslateCmd = maxTranslateCmd;
            this.kPAim = kPAim;
            this.maxOmegaCmd = maxOmegaCmd;
            this.minOmegaCmd = minOmegaCmd;
            this.aimDeadbandRad = aimDeadbandRad;
        }

        /**
         * Return a copy of this tuning with a different translation kP gain.
         *
         * <p>Higher values make the robot drive toward the translation target more aggressively.</p>
         */
        public Tuning withTranslateKp(double kPTranslate) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different maximum translation command.
         *
         * <p>This caps how fast the assist can drive the robot in X/Y (typical range: 0..1).</p>
         */
        public Tuning withMaxTranslateCmd(double maxTranslateCmd) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different aim (turn) kP gain.
         *
         * <p>Higher values make the robot turn toward the aim target more aggressively.</p>
         */
        public Tuning withAimKp(double kPAim) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different maximum omega (turn) command.
         *
         * <p>This caps how fast the assist can turn the robot (typical range: 0..1).</p>
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
         */
        public Tuning withMinOmegaCmd(double minOmegaCmd) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different aim deadband in radians.
         *
         * <p>If the aim error magnitude is smaller than this, DriveGuidance outputs 0 turn command.</p>
         */
        public Tuning withAimDeadbandRad(double aimDeadbandRad) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, minOmegaCmd, aimDeadbandRad);
        }

        /**
         * Convenience overload: set the aim deadband in degrees.
         */
        public Tuning withAimDeadbandDeg(double aimDeadbandDeg) {
            return withAimDeadbandRad(Math.toRadians(aimDeadbandDeg));
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
     * Alias for {@link #requestedMask()}.
     */
    public DriveOverlayMask suggestedMask() {
        return spec.suggestedMask();
    }

    /**
     * Create a {@link DriveOverlay} implementing this plan.
     */
    public DriveOverlay overlay() {
        return new DriveGuidanceOverlay(this);
    }

    /**
     * Build a {@link DriveGuidanceTask} that executes this plan as an autonomous-style task.
     */
    public DriveGuidanceTask task(MecanumDrivebase drivebase, DriveGuidanceTask.Config cfg) {
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
