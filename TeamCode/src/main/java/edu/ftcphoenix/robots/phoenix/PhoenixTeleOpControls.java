package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.BindingRegistrar;

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

    private final PhoenixProfile.TeleOpControlsConfig cfg;
    private final BindingRegistrar bindingRegistrar;
    private final DriveSource manualDrive;
    private final ScalarSource manualTranslateMagnitude;
    private final BooleanSource autoAimEnabled;
    private final BooleanSource aimOverride;
    private final GamepadDevice driver;
    private final GamepadDevice operator;

    /**
     * Creates the Phoenix TeleOp controls owner.
     *
     * <p>
     * The constructor only establishes the stable driver/operator sources. Mechanism-specific button
     * bindings are registered later through {@link #bind(PhoenixCapabilities)} once the shared
     * robot capability families exist.
     * </p>
     *
     * @param bindingRegistrar managed program registration surface that owns no independent
     *                         heartbeat in this controls object
     * @param gamepads         wrapped gamepad sources used to map driver/operator controls
     * @param config           TeleOp control-layer config; defensively copied for local ownership
     */
    public PhoenixTeleOpControls(BindingRegistrar bindingRegistrar,
                                 Gamepads gamepads,
                                 PhoenixProfile.TeleOpControlsConfig config) {
        this.bindingRegistrar = Objects.requireNonNull(
                bindingRegistrar,
                "bindingRegistrar"
        );
        Objects.requireNonNull(gamepads, "gamepads");
        this.cfg = Objects.requireNonNull(config, "config").copy();

        this.driver = gamepads.p1();
        this.operator = gamepads.p2();
        this.manualTranslateMagnitude = driver.leftStickMagnitude().memoized();

        manualDrive = new GamepadDriveSource(
                driver.leftX(),
                driver.leftY(),
                driver.rightX(),
                this.cfg.drive.manualDrive
        ).scaledWhen(
                driver.rightBumper(),
                this.cfg.drive.slowTranslateScale,
                this.cfg.drive.slowOmegaScale
        ).rateLimited(
                this.cfg.drive.maxAxialRatePerSec,
                this.cfg.drive.maxLateralRatePerSec,
                this.cfg.drive.maxOmegaRatePerSec
        );

        autoAimEnabled = operator.leftBumper().memoized();
        aimOverride = operator.y().memoized();
    }

    /**
     * Registers the Phoenix scoring control semantics on the managed binding registrar.
     *
     * <p>
     * TeleOp binds against the shared robot capability families instead of directly depending on
     * Phoenix internals. That keeps the control layer mode-neutral and leaves room for Auto to use
     * the same vocabulary through tasks instead of button bindings. Future frame-valued manual
     * mechanism commands should also be registered here through
     * {@code BindingRegistrar.copyEachCycle(...)}. The managed program retains the one heartbeat
     * and cleanup owner for every declaration made here.
     * </p>
     *
     * @param capabilities shared Phoenix capability families exposed by the robot container
     */
    public void bind(PhoenixCapabilities capabilities) {
        PhoenixCapabilities.Scoring scoring =
                Objects.requireNonNull(capabilities, "capabilities").scoring();

        bindingRegistrar.toggleOnRise(operator.a(), scoring::setIntakeEnabled);
        bindingRegistrar.toggleOnRise(operator.rightBumper(), scoring::setFlywheelEnabled);

        // Declaration order is intentional: if capture and a nudge rise in the same input frame,
        // capture the suggested velocity first and then apply the student's selected adjustment.
        bindingRegistrar.onRise(operator.leftBumper(), scoring::captureSuggestedShotVelocity);

        bindingRegistrar.mirrorOnChange(operator.b(), scoring::setShootingEnabled);
        bindingRegistrar.mirrorOnChange(operator.x(), scoring::setEjectEnabled);

        bindingRegistrar.nudgeOnRise(
                operator.dpadUp(),
                operator.dpadDown(),
                cfg.selectedVelocityStepNative,
                scoring::adjustSelectedVelocityNative
        );
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
