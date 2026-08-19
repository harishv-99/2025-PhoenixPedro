package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.ftc.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.CallbackBindings;

/**
 * Phoenix TeleOp controls owner.
 *
 * <p>
 * This object owns <em>all</em> TeleOp input semantics for Phoenix, including manual drive axes,
 * slow mode, auto-aim enable/override sources, and scoring button bindings. Keeping those choices
 * together makes it obvious where driver policy lives and avoids scattering drive control setup in
 * one class while scoring bindings live in another.
 * </p>
 *
 * <p>
 * The framework drive owner and localization lane own stable hardware/resource behavior. This
 * controls object owns operator intent mapping. That split is intentional and is the pattern
 * future robots should follow too.
 * </p>
 */
public final class PhoenixTeleOpControls {

    /** Mutable data-only tuning for Phoenix's TeleOp control layer. */
    public static final class Config {

        /** Stick shaping and scaling for the base manual drive source. */
        public GamepadDriveSource.Config manualDrive = GamepadDriveSource.Config.defaults();

        /** Translation scale applied while the driver holds slow mode. */
        public double slowTranslateScale = 0.35;

        /** Maximum axial command change per second. */
        public double maxAxialRatePerSec = 4.0;

        /** Maximum lateral command change per second. */
        public double maxLateralRatePerSec = 4.0;

        /** Maximum rotational command change per second. */
        public double maxOmegaRatePerSec = 6.0;

        /** Rotation scale applied while the driver holds slow mode. */
        public double slowOmegaScale = 0.20;

        /** Step used when the operator nudges the selected flywheel velocity. */
        public double selectedVelocityStepNative = 25.0;

        private Config() {
        }

        /**
         * Returns a fresh complete software-valid Phoenix controls draft.
         *
         * @return fresh mutable controls configuration
         */
        public static Config defaults() {
            return new Config();
        }
    }

    private final DriveSource manualDrive;
    private final ScalarSource manualTranslateMagnitude;
    private final BooleanSource autoAimEnabled;
    private final BooleanSource aimOverride;
    private final GamepadDevice driver;
    private final GamepadDevice operator;
    private final double selectedVelocityStepNative;
    private boolean bindAttempted;

    /**
     * Creates the Phoenix TeleOp controls owner.
     *
     * <p>
     * The constructor only establishes the stable driver/operator sources. Mechanism-specific button
     * bindings are registered later through {@link #bind(CallbackBindings, PhoenixCapabilities)}
     * once the shared robot capability families exist.
     * </p>
     *
     * @param gamepads wrapped gamepad sources used to map driver/operator controls
     * @param config TeleOp control-layer draft; retained scalar values are captured, and the
     *               framework manual-drive owner defensively copies its nested draft
     * @throws NullPointerException if an argument or {@link Config#manualDrive} is {@code null}
     * @throws IllegalArgumentException if an outer scalar is non-finite or outside its documented
     *                                  domain, or the nested manual-drive draft is invalid
     */
    public PhoenixTeleOpControls(Gamepads gamepads, Config config) {
        Gamepads requiredGamepads = Objects.requireNonNull(gamepads, "gamepads");
        Config draft = Objects.requireNonNull(config, "PhoenixTeleOpControls.Config");
        GamepadDriveSource.Config manualDriveConfig = Objects.requireNonNull(
                draft.manualDrive,
                "PhoenixTeleOpControls.Config.manualDrive"
        );
        double slowTranslateScale = requireFiniteUnitInterval(
                "slowTranslateScale",
                draft.slowTranslateScale
        );
        double maxAxialRatePerSec = requireFinitePositive(
                "maxAxialRatePerSec",
                draft.maxAxialRatePerSec
        );
        double maxLateralRatePerSec = requireFinitePositive(
                "maxLateralRatePerSec",
                draft.maxLateralRatePerSec
        );
        double maxOmegaRatePerSec = requireFinitePositive(
                "maxOmegaRatePerSec",
                draft.maxOmegaRatePerSec
        );
        double slowOmegaScale = requireFiniteUnitInterval(
                "slowOmegaScale",
                draft.slowOmegaScale
        );
        this.selectedVelocityStepNative = requireFinitePositive(
                "selectedVelocityStepNative",
                draft.selectedVelocityStepNative
        );

        this.driver = requiredGamepads.p1();
        this.operator = requiredGamepads.p2();
        this.manualTranslateMagnitude = driver.leftStickMagnitude().memoized();

        manualDrive = new GamepadDriveSource(
                driver.leftX(),
                driver.leftY(),
                driver.rightX(),
                manualDriveConfig
        ).scaledWhen(
                driver.rightBumper(),
                slowTranslateScale,
                slowOmegaScale
        ).rateLimited(
                maxAxialRatePerSec,
                maxLateralRatePerSec,
                maxOmegaRatePerSec
        );

        autoAimEnabled = operator.leftBumper().memoized();
        aimOverride = operator.y().memoized();
    }

    /**
     * Declares the Phoenix scoring control semantics on the managed callback surface.
     *
     * <p>
     * TeleOp binds against the shared robot capability families instead of directly depending on
     * Phoenix internals. That keeps the control layer mode-neutral and leaves room for Auto to use
     * the same vocabulary through tasks instead of button bindings. Future frame-valued manual
     * mechanism commands should also be registered here through
     * {@code CallbackBindings.copyEachCycle(...)}. The managed program retains the one heartbeat
     * and cleanup owner for every declaration made here.
     * </p>
     *
     * <p>This declaration is single-use. A repeated call, including a retry after a callback
     * surface rejected part of the first declaration, fails before adding another registration.</p>
     *
     * @param callbackBindings managed callback surface that owns no independent heartbeat here
     * @param capabilities shared Phoenix capability families exposed by the robot container
     * @throws NullPointerException if an argument or its scoring family is {@code null}; this does
     *                              not consume the bind opportunity
     * @throws IllegalStateException if a bind was already attempted, including one whose callback
     *                               registration failed partway through
     */
    public void bind(CallbackBindings callbackBindings, PhoenixCapabilities capabilities) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(
                callbackBindings,
                "callbackBindings"
        );
        PhoenixCapabilities requiredCapabilities = Objects.requireNonNull(
                capabilities,
                "capabilities"
        );
        PhoenixCapabilities.Scoring scoring = Objects.requireNonNull(
                requiredCapabilities.scoring(),
                "capabilities.scoring()"
        );
        claimBind();

        requiredCallbacks.toggleOnRise(operator.a(), scoring::setIntakeEnabled);
        requiredCallbacks.toggleOnRise(operator.rightBumper(), scoring::setFlywheelEnabled);

        // Declaration order is intentional: if capture and a nudge rise in the same input frame,
        // capture the suggested velocity first and then apply the student's selected adjustment.
        requiredCallbacks.onRise(operator.leftBumper(), scoring::captureSuggestedShotVelocity);

        requiredCallbacks.mirrorOnChange(operator.b(), scoring::setShootingEnabled);
        requiredCallbacks.mirrorOnChange(operator.x(), scoring::setEjectEnabled);

        requiredCallbacks.nudgeOnRise(
                operator.dpadUp(),
                operator.dpadDown(),
                selectedVelocityStepNative,
                scoring::adjustSelectedVelocityNative
        );
    }

    private static double requireFiniteUnitInterval(String fieldName, double value) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    "PhoenixTeleOpControls.Config." + fieldName
                            + " must be finite and in [0.0, 1.0], got " + value
            );
        }
        return value;
    }

    private static double requireFinitePositive(String fieldName, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    "PhoenixTeleOpControls.Config." + fieldName
                            + " must be finite and > 0.0, got " + value
            );
        }
        return value;
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "PhoenixTeleOpControls.bind(...) may be called only once; "
                            + "create a fresh controls owner for another callback graph"
            );
        }
        bindAttempted = true;
    }

    /**
     * Returns the manual drive source owned by this controls object.
     *
     * @return manual drive source built from the configured driver axes and slow-mode policy
     */
    public DriveSource manualDriveSource() {
        return manualDrive;
    }

    /**
     * Returns the current manual translation-stick magnitude as a source.
     *
     * <p>
     * Higher-level robot services can depend on this source when they need to reason about driver
     * translation intent without taking ownership of button or stick semantics themselves.
     * </p>
     *
     * @return memoized scalar source representing normalized translation-stick magnitude in [0, 1]
     */
    public ScalarSource manualTranslateMagnitudeSource() {
        return manualTranslateMagnitude;
    }


    /**
     * Returns the operator-controlled auto-aim enable source.
     *
     * @return memoized boolean source that becomes true while auto aim is enabled
     */
    public BooleanSource autoAimEnabledSource() {
        return autoAimEnabled;
    }

    /**
     * Returns the operator-controlled aim-override source.
     *
     * @return memoized boolean source that becomes true while override is held
     */
    public BooleanSource aimOverrideSource() {
        return aimOverride;
    }

    /**
     * Emits the standard Phoenix TeleOp control help block during INIT.
     *
     * @param telemetry FTC telemetry sink to write to; ignored when {@code null}
     */
    public void emitInitHelp(Telemetry telemetry) {
        if (telemetry == null) {
            return;
        }
        telemetry.addLine("Phoenix TeleOp (framework lanes + controls + drive assists + targeting + presenter)");
        telemetry.addLine("P1: Left stick=drive, Right stick=turn, RB=slow mode");
        telemetry.addLine("P2: RB=toggle shooter flywheel (spins at selected velocity)");
        telemetry.addLine("P2: LB=auto aim + set velocity from AprilTag range");
        telemetry.addLine("P2: B=shoot (hold; release cancels)");
        telemetry.addLine("P2: Y=override shoot gates (hold; if flywheel ON, forces feed even if not ready)");
        telemetry.addLine("P2: A=toggle intake");
        telemetry.addLine("P2: X=eject / unjam (hold; reverse feeds)");
        telemetry.addLine("P2: DPad Up/Down=adjust selected velocity");
    }

}
