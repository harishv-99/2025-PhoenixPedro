package edu.ftcphoenix.fw.tools.tester.calibration;

import java.util.Locale;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;

/**
 * Step-0 bring-up tester for goBILDA Pinpoint odometry:
 * verify Phoenix axis conventions match your physical robot motion.
 *
 * <p>Phoenix conventions (robot/field planar pose):</p>
 * <ul>
 *   <li><b>+X</b> forward</li>
 *   <li><b>+Y</b> left</li>
 *   <li><b>heading</b> is CCW-positive</li>
 * </ul>
 *
 * <p>This tester guides you through three short samples (forward, left, rotate CCW)
 * and suggests which {@link PinpointPoseEstimator.Config} fields to flip when signs
 * are inverted.</p>
 *
 * <h2>Controls (gamepad1)</h2>
 * <ul>
 *   <li><b>X</b>: Reset pose + clear results</li>
 *   <li><b>A</b>: Start/stop Forward sample (push robot forward by hand)</li>
 *   <li><b>Y</b>: Start/stop Left sample (push robot left by hand)</li>
 *   <li><b>B</b>: Start/stop Rotate sample (rotate robot CCW by hand)</li>
 * </ul>
 */
public final class PinpointAxisDirectionTester extends BaseTeleOpTester {

    /**
     * Configuration for the tester.
     */
    public static final class Config {

        /**
         * Pinpoint estimator configuration (device name, offsets, pod directions, etc).
         */
        public PinpointPoseEstimator.Config pinpoint = PinpointPoseEstimator.Config.defaults();

        /**
         * Minimum translation (inches) required to treat a sample as valid.
         */
        public double minTranslationInches = 6.0;

        /**
         * Minimum rotation (degrees) required to treat the rotation sample as valid.
         */
        public double minRotationDeg = 20.0;

        private Config() {
        }

        public static Config defaults() {
            return new Config();
        }
    }

    private enum Mode {
        IDLE,
        SAMPLE_FORWARD,
        SAMPLE_LEFT,
        SAMPLE_ROTATE
    }

    private final Config cfg;

    private PinpointPoseEstimator pinpoint;

    private Mode mode = Mode.IDLE;
    private Pose2d startPose = Pose2d.zero();

    // Unwrapped heading for the rotation sample.
    private final AngleUnwrapper headingUnwrapper = new AngleUnwrapper();
    private double startHeadingUnwrappedRad = 0.0;

    // Last completed sample results.
    private SampleResult forwardResult = null;
    private SampleResult leftResult = null;
    private SampleResult rotateResult = null;

    public PinpointAxisDirectionTester() {
        this(Config.defaults());
    }

    public PinpointAxisDirectionTester(Config cfg) {
        this.cfg = (cfg != null) ? cfg : Config.defaults();
    }

    @Override
    public String name() {
        return "Pinpoint: Axis Direction Check";
    }

    @Override
    protected void onInit() {
        pinpoint = new PinpointPoseEstimator(ctx.hw, cfg.pinpoint);

        // Controls.
        bindings.onPress(gamepads.p1().x(), this::resetAndClear);
        bindings.onPress(gamepads.p1().a(), () -> toggleSample(Mode.SAMPLE_FORWARD));
        bindings.onPress(gamepads.p1().y(), () -> toggleSample(Mode.SAMPLE_LEFT));
        bindings.onPress(gamepads.p1().b(), () -> toggleSample(Mode.SAMPLE_ROTATE));

        resetAndClear();
    }


    @Override
    protected void onInitLoop(double dtSec) {
        // Render during INIT as well (tester suite allows entering testers before Start).
        // Without this, the last menu telemetry can appear "frozen" when you enter this tester
        // before pressing Start.
        if (pinpoint != null) {
            pinpoint.update(clock);
        }

        Pose2d pose = latestPose();

        // Update heading unwrapper during rotation sampling.
        if (mode == Mode.SAMPLE_ROTATE) {
            headingUnwrapper.update(pose.headingRad);
        }

        render(pose);
    }

    @Override
    protected void onLoop(double dtSec) {
        // Keep Pinpoint updated.
        if (pinpoint != null) {
            pinpoint.update(clock);
        }

        Pose2d pose = latestPose();

        // Update heading unwrapper during rotation sampling.
        if (mode == Mode.SAMPLE_ROTATE) {
            headingUnwrapper.update(pose.headingRad);
        }

        // Render UI.
        render(pose);
    }

    // ---------------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------------

    private void resetAndClear() {
        mode = Mode.IDLE;
        startPose = Pose2d.zero();
        headingUnwrapper.reset(0.0);
        startHeadingUnwrappedRad = 0.0;

        forwardResult = null;
        leftResult = null;
        rotateResult = null;

        if (pinpoint != null) {
            pinpoint.setPose(Pose2d.zero());
        }
    }

    private void toggleSample(Mode requested) {
        if (requested == null) return;

        if (mode == requested) {
            // Stop sample.
            finishSample();
            mode = Mode.IDLE;
            return;
        }

        // Starting a new sample cancels the old one.
        mode = requested;
        startPose = latestPose();

        if (mode == Mode.SAMPLE_ROTATE) {
            headingUnwrapper.reset(startPose.headingRad);
            startHeadingUnwrappedRad = headingUnwrapper.getUnwrappedRad();
        }
    }

    private void finishSample() {
        Pose2d endPose = latestPose();

        double dx = endPose.xInches - startPose.xInches;
        double dy = endPose.yInches - startPose.yInches;

        double dHeadingRad;
        if (mode == Mode.SAMPLE_ROTATE) {
            // Use unwrapped heading change to avoid wrap-around artifacts.
            headingUnwrapper.update(endPose.headingRad);
            dHeadingRad = headingUnwrapper.getUnwrappedRad() - startHeadingUnwrappedRad;
        } else {
            dHeadingRad = MathUtil.wrapToPi(endPose.headingRad - startPose.headingRad);
        }

        SampleResult r = new SampleResult(dx, dy, dHeadingRad);

        switch (mode) {
            case SAMPLE_FORWARD:
                forwardResult = r;
                break;
            case SAMPLE_LEFT:
                leftResult = r;
                break;
            case SAMPLE_ROTATE:
                rotateResult = r;
                break;
            case IDLE:
            default:
                // no-op
                break;
        }
    }

    private Pose2d latestPose() {
        if (pinpoint == null) {
            return Pose2d.zero();
        }
        PoseEstimate est = pinpoint.getEstimate();
        if (est == null || !est.hasPose) {
            return Pose2d.zero();
        }
        return est.toPose2d();
    }

    private void render(Pose2d pose) {
        telemHeader("Pinpoint Axis Direction Check");

        ctx.telemetry.addLine("Phoenix pose convention: +X forward, +Y left, heading CCW+.");
        ctx.telemetry.addLine("Do each sample by hand, then press the same button again to stop.");
        ctx.telemetry.addLine("");

        ctx.telemetry.addData("mode", mode);
        ctx.telemetry.addData("pose.x(in)", fmt(pose.xInches));
        ctx.telemetry.addData("pose.y(in)", fmt(pose.yInches));
        ctx.telemetry.addData("pose.heading(deg)", fmt(Math.toDegrees(pose.headingRad)));
        ctx.telemetry.addLine("");

        // Current config snapshot.
        PinpointPoseEstimator.Config pcfg = cfg.pinpoint;
        if (pcfg != null) {
            ctx.telemetry.addLine("Current Pinpoint config:");
            ctx.telemetry.addData("pinpoint.hardwareMapName", pcfg.hardwareMapName);
            ctx.telemetry.addData("pinpoint.forwardPodDirection", pcfg.forwardPodDirection);
            ctx.telemetry.addData("pinpoint.strafePodDirection", pcfg.strafePodDirection);
            ctx.telemetry.addData("pinpoint.yawScalar", pcfg.yawScalar);
            ctx.telemetry.addLine("");
        }

        // Live delta while sampling.
        if (mode != Mode.IDLE) {
            double dx = pose.xInches - startPose.xInches;
            double dy = pose.yInches - startPose.yInches;
            double dh;
            if (mode == Mode.SAMPLE_ROTATE) {
                dh = headingUnwrapper.getUnwrappedRad() - startHeadingUnwrappedRad;
            } else {
                dh = MathUtil.wrapToPi(pose.headingRad - startPose.headingRad);
            }
            ctx.telemetry.addLine("--- ACTIVE SAMPLE ---");
            ctx.telemetry.addData("delta.x(in)", fmt(dx));
            ctx.telemetry.addData("delta.y(in)", fmt(dy));
            ctx.telemetry.addData("delta.heading(deg)", fmt(Math.toDegrees(dh)));
            ctx.telemetry.addLine("");
        }

        // Completed results + suggestions.
        ctx.telemetry.addLine("--- RESULTS ---");
        renderForward();
        renderLeft();
        renderRotate();

        ctx.telemetry.addLine("");
        ctx.telemetry.addLine("Controls: X reset | A forward | Y left | B rotate CCW");
        telemUpdate();
    }

    private void renderForward() {
        ctx.telemetry.addLine("Forward sample (A): push robot forward");
        if (forwardResult == null) {
            ctx.telemetry.addLine("  (no sample yet)");
            return;
        }

        double dx = forwardResult.dxIn;
        ctx.telemetry.addData("  dx(in)", fmt(dx));

        if (Math.abs(dx) < cfg.minTranslationInches) {
            ctx.telemetry.addLine("  -> Move farther (at least ~" + fmt(cfg.minTranslationInches) + " in)");
            return;
        }

        if (dx > 0) {
            ctx.telemetry.addLine("  -> OK: +X is forward");
        } else {
            ctx.telemetry.addLine("  -> WRONG SIGN: forward produced negative X");
            ctx.telemetry.addLine("     Suggest: cfg.pinpoint.withForwardPodDirection(REVERSE)");
        }
    }

    private void renderLeft() {
        ctx.telemetry.addLine("Left sample (Y): push robot left");
        if (leftResult == null) {
            ctx.telemetry.addLine("  (no sample yet)");
            return;
        }

        double dy = leftResult.dyIn;
        ctx.telemetry.addData("  dy(in)", fmt(dy));

        if (Math.abs(dy) < cfg.minTranslationInches) {
            ctx.telemetry.addLine("  -> Move farther (at least ~" + fmt(cfg.minTranslationInches) + " in)");
            return;
        }

        if (dy > 0) {
            ctx.telemetry.addLine("  -> OK: +Y is left");
        } else {
            ctx.telemetry.addLine("  -> WRONG SIGN: left produced negative Y");
            ctx.telemetry.addLine("     Suggest: cfg.pinpoint.withStrafePodDirection(REVERSE)");
        }
    }

    private void renderRotate() {
        ctx.telemetry.addLine("Rotate sample (B): rotate robot CCW");
        if (rotateResult == null) {
            ctx.telemetry.addLine("  (no sample yet)");
            return;
        }

        double dDeg = Math.toDegrees(rotateResult.dHeadingRad);
        ctx.telemetry.addData("  dHeading(deg)", fmt(dDeg));

        if (Math.abs(dDeg) < cfg.minRotationDeg) {
            ctx.telemetry.addLine("  -> Rotate more (at least ~" + fmt(cfg.minRotationDeg) + " deg)");
            return;
        }

        if (dDeg > 0) {
            ctx.telemetry.addLine("  -> OK: heading is CCW-positive");
        } else {
            ctx.telemetry.addLine("  -> WRONG SIGN: CCW rotation produced negative heading");
            ctx.telemetry.addLine("     Check Pinpoint mounting orientation / IMU alignment.");
            ctx.telemetry.addLine("     If supported, you can also try setting yawScalar to a negative value.");
        }
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    private static final class SampleResult {
        final double dxIn;
        final double dyIn;
        final double dHeadingRad;

        SampleResult(double dxIn, double dyIn, double dHeadingRad) {
            this.dxIn = dxIn;
            this.dyIn = dyIn;
            this.dHeadingRad = dHeadingRad;
        }
    }

    /**
     * Tiny helper to unwrap heading across the (-pi, pi] wrap.
     */
    private static final class AngleUnwrapper {
        private boolean initialized = false;
        private double prevRad = 0.0;
        private double unwrappedRad = 0.0;

        public void reset(double initialRad) {
            initialized = true;
            prevRad = initialRad;
            unwrappedRad = initialRad;
        }

        public void update(double currentRad) {
            if (!initialized) {
                reset(currentRad);
                return;
            }
            double d = MathUtil.wrapToPi(currentRad - prevRad);
            unwrappedRad += d;
            prevRad = currentRad;
        }

        public double getUnwrappedRad() {
            return unwrappedRad;
        }
    }
}
