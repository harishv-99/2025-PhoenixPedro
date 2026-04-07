package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;

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
 * The framework drive and localization lanes own stable hardware/resource behavior. This controls
 * object owns operator intent mapping. That split is intentional and is the pattern future robots
 * should follow too.
 * </p>
 */
public final class PhoenixTeleOpControls {

    private final PhoenixProfile.TeleOpControlsConfig cfg;
    private final Bindings bindings = new Bindings();
    private final DriveSource manualDrive;
    private final BooleanSource autoAimEnabled;
    private final BooleanSource aimOverride;
    private final GamepadDevice driver;
    private final GamepadDevice operator;

    /**
     * Creates the Phoenix TeleOp controls owner.
     *
     * <p>
     * The constructor only establishes the stable driver/operator sources. Mechanism-specific button
     * bindings are registered later through {@link #bindScoringControls(Shooter, ShooterSupervisor, Runnable)}
     * once the scoring collaborators exist.
     * </p>
     *
     * @param gamepads wrapped gamepad sources used to map driver/operator controls
     * @param config TeleOp control-layer config; defensively copied for local ownership
     */
    public PhoenixTeleOpControls(Gamepads gamepads,
                                 PhoenixProfile.TeleOpControlsConfig config) {
        Objects.requireNonNull(gamepads, "gamepads");
        this.cfg = Objects.requireNonNull(config, "config").copy();

        this.driver = gamepads.p1();
        this.operator = gamepads.p2();

        manualDrive = new GamepadDriveSource(
                driver.leftX(),
                driver.leftY(),
                driver.rightX(),
                this.cfg.drive.manualDrive
        ).scaledWhen(
                driver.rightBumper(),
                this.cfg.drive.slowTranslateScale,
                this.cfg.drive.slowOmegaScale
        );

        autoAimEnabled = operator.leftBumper().memoized();
        aimOverride = operator.y().memoized();
    }

    /**
     * Registers the Phoenix scoring button semantics with this controls owner.
     *
     * @param shooter shooter subsystem used for direct selected-velocity adjustments
     * @param scoring scoring supervisor that receives intent-level actions
     * @param captureSuggestedVelocity action invoked when the operator requests a fresh
     *                                 target-based velocity suggestion
     */
    public void bindScoringControls(Shooter shooter,
                                    ShooterSupervisor scoring,
                                    Runnable captureSuggestedVelocity) {
        Objects.requireNonNull(shooter, "shooter");
        Objects.requireNonNull(scoring, "scoring");
        Objects.requireNonNull(captureSuggestedVelocity, "captureSuggestedVelocity");

        bindings.onToggle(operator.a(), scoring::setIntakeEnabled);
        bindings.onToggle(operator.rightBumper(), scoring::setFlywheelEnabled);

        bindings.onRise(operator.leftBumper(), captureSuggestedVelocity);

        bindings.onRiseAndFall(
                operator.b(),
                scoring::beginShooting,
                scoring::endShooting
        );

        bindings.onRiseAndFall(
                operator.x(),
                scoring::beginEject,
                scoring::endEject
        );

        bindings.onRise(operator.dpadUp(), shooter::increaseSelectedVelocity);
        bindings.onRise(operator.dpadDown(), shooter::decreaseSelectedVelocity);
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
     * Returns the current manual translation-stick magnitude.
     *
     * <p>
     * Phoenix uses this to decide when the driver is still actively translating during shoot-brace
     * behavior. Exposing it here keeps drive-input semantics inside the controls owner.
     * </p>
     *
     * @param clock shared loop clock used to sample the stick magnitude source
     * @return current left-stick magnitude in [0, 1]
     */
    public double manualTranslateMagnitude(LoopClock clock) {
        return driver.leftStickMagnitude().getAsDouble(clock);
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
        telemetry.addLine("Phoenix TeleOp (framework lanes + controls + targeting + presenter)");
        telemetry.addLine("P1: Left stick=drive, Right stick=turn, RB=slow mode");
        telemetry.addLine("P2: RB=toggle shooter flywheel (spins at selected velocity)");
        telemetry.addLine("P2: LB=auto aim + set velocity from AprilTag range");
        telemetry.addLine("P2: B=shoot (hold; release cancels)");
        telemetry.addLine("P2: Y=override shoot gates (hold; if flywheel ON, forces feed even if not ready)");
        telemetry.addLine("P2: A=toggle intake");
        telemetry.addLine("P2: X=eject / unjam (hold; reverse feeds)");
        telemetry.addLine("P2: DPad Up/Down=adjust selected velocity");
    }

    /**
     * Advances the binding state machine for the current loop cycle.
     *
     * @param clock shared loop clock for the active OpMode cycle
     */
    public void update(LoopClock clock) {
        bindings.update(clock);
    }

    /**
     * Clears all remembered edge/toggle state.
     *
     * <p>
     * Call this when TeleOp is shutting down or when the controls owner is being discarded so the
     * next activation starts from a clean slate.
     * </p>
     */
    public void clear() {
        bindings.clear();
    }
}
