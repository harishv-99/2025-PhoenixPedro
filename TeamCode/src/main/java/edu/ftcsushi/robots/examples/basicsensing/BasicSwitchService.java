package edu.ftcsushi.robots.examples.basicsensing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;

/**
 * Owns one active-low switch observation and publishes cached status without commanding motion.
 *
 * <p>Register this owner once as a managed service. Construction configures the input but does
 * not read it; START resets and samples it, and each active service phase refreshes its status.
 * The raw and conditioned sources share one successful electrical read per clock cycle. A failed
 * read publishes no new status; the managed host then stops the service and becomes terminal.</p>
 */
public final class BasicSwitchService implements RobotProgram.Service {

    /** Data-only input name and sampled debounce delays; the owner snapshots every field. */
    public static final class Config {
        /** Nonblank FTC digital-channel name; the reviewed wiring must make LOW mean pressed. */
        public String switchName;
        /** Finite, nonnegative sampled debounce delay for becoming pressed, in seconds. */
        public double pressedDebounceSec;
        /** Finite, nonnegative sampled debounce delay for becoming released, in seconds. */
        public double releasedDebounceSec;

        private Config() {
        }

        /** Returns the lesson's one complete software baseline, not evidence of physical wiring. */
        public static Config defaults() {
            Config config = new Config();
            config.switchName = "lessonSwitch";
            config.pressedDebounceSec = 0.02;
            config.releasedDebounceSec = 0.02;
            return config;
        }
    }

    /** Immutable facts from the last complete successful sample, or an unobserved lifecycle state. */
    public static final class Status {
        /** Whether this lifetime has a cached observation; false makes the other fields unknown. */
        public final boolean observed;
        /** The sampled LOW-as-pressed meaning before debounce; it is not the raw HIGH pin level. */
        public final boolean rawPressed;
        /** The sampled pressed meaning after the configured on/off debounce. */
        public final boolean pressed;

        private Status(boolean observed, boolean rawPressed, boolean pressed) {
            this.observed = observed;
            this.rawPressed = rawPressed;
            this.pressed = pressed;
        }
    }

    private static final Status NOT_OBSERVED = new Status(false, false, false);

    private final BooleanSource rawPressedSource;
    private final BooleanSource pressedSource;
    private Status status = NOT_OBSERVED;

    /**
     * Validates and snapshots configuration before resolving and configuring the one input.
     *
     * @param hardwareMap FTC registry, or the software-device registry in a focused test
     * @param config complete input name and delays; later edits cannot change this owner
     * @throws NullPointerException if either argument is null
     * @throws IllegalArgumentException if the name is blank or a delay is nonfinite or negative
     */
    public BasicSwitchService(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config required = Objects.requireNonNull(config, "BasicSwitchService.Config is required");
        String switchName = requireName(required.switchName);
        double pressedDelay = requireDelay(required.pressedDebounceSec, "pressedDebounceSec");
        double releasedDelay = requireDelay(required.releasedDebounceSec, "releasedDebounceSec");

        rawPressedSource = FtcSensors.digitalLow(map, switchName);
        pressedSource = rawPressedSource.debouncedOnOff(pressedDelay, releasedDelay);
    }

    /** Returns the cached immutable status without reading the input or advancing debounce. */
    public Status status() {
        return status;
    }

    /** Resets the graph and samples at the managed START boundary, whose elapsed interval is zero. */
    @Override
    public void start(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        pressedSource.reset();
        status = NOT_OBSERVED;
        update(clock);
    }

    /**
     * Publishes raw and conditioned facts from one successful cycle observation.
     * Repeated same-cycle calls reuse the source results; failure leaves the prior status intact.
     */
    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        boolean rawPressed = rawPressedSource.getAsBoolean(clock);
        boolean pressed = pressedSource.getAsBoolean(clock);
        status = new Status(true, rawPressed, pressed);
    }

    /**
     * Clears cached evidence and resets the graph without reading hardware. Safe before START and
     * on repeated cleanup; the managed host prevents further updates after STOP.
     */
    @Override
    public void stop() {
        status = NOT_OBSERVED;
        pressedSource.reset();
    }

    /** Validates and normalizes the configured name before any hardware lookup. */
    private static String requireName(String name) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("BasicSwitchService.Config.switchName must not be blank");
        }
        return name.trim();
    }

    /** Rejects an unusable sampled delay with the exact configuration field to correct. */
    private static double requireDelay(double delay, String field) {
        if (!Double.isFinite(delay) || delay < 0.0) {
            throw new IllegalArgumentException(
                    "BasicSwitchService.Config." + field + " must be finite and >= 0, got " + delay);
        }
        return delay;
    }
}
