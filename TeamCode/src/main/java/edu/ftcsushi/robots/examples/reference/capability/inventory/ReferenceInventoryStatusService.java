package edu.ftcsushi.robots.examples.reference.capability.inventory;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;

/**
 * Optional Reference case study that derives one coherent inventory status from three ordered
 * active-low position sensors.
 *
 * <p>This is robot-owned example code, not a generic framework inventory abstraction. Declare one
 * instance as an upstream {@link RobotProgram.Service}; presenters may read {@link #status()} and
 * Tasks may observe the stable cached {@link #fullSource()} without causing hardware reads.</p>
 *
 * <p>The service samples once at START and once in each managed service phase. Its source graphs
 * provide same-cycle hardware-read caching and debounce lifecycle. Publication is atomic: a failed
 * three-sensor observation leaves the prior immutable snapshot in place and may retry that cycle.
 * Repeating a successful update preserves the exact publication, not a newly stamped copy.</p>
 */
public final class ReferenceInventoryStatusService implements RobotProgram.Service {

    /** Mutable data-only configuration; the service validates and snapshots every field. */
    public static final class Config {
        public String firstPositionSensorName;
        public String secondPositionSensorName;
        public String thirdPositionSensorName;
        public double occupiedDebounceSec;
        public double vacatedDebounceSec;

        private Config() {
        }

        /** Returns a software baseline whose physical names and timings must be reviewed. */
        public static Config defaults() {
            Config config = new Config();
            config.firstPositionSensorName = "inventoryFirst";
            config.secondPositionSensorName = "inventorySecond";
            config.thirdPositionSensorName = "inventoryThird";
            config.occupiedDebounceSec = 0.02;
            config.vacatedDebounceSec = 0.02;
            return config;
        }
    }

    /** Describes an observation that does not match the Reference example's ordered-fill model. */
    public enum OrderIssue {
        NONE,
        SECOND_WITHOUT_FIRST,
        THIRD_WITHOUT_SECOND
    }

    /** Immutable status derived from one complete successful three-sensor observation. */
    public static final class Status {
        public final boolean observed;
        public final boolean firstPositionOccupied;
        public final boolean secondPositionOccupied;
        public final boolean thirdPositionOccupied;
        public final int conditionedOccupiedPositionCount;
        public final boolean full;
        public final OrderIssue orderIssue;
        /** Successful software sampling time, not a native sensor-frame acquisition timestamp. */
        public final LoopTimestamp sampledAt;
        /** Successful sampling cycle, or {@code -1} before observation and after STOP. */
        public final long sampleCycle;

        private Status(boolean observed, boolean first, boolean second, boolean third,
                       LoopTimestamp sampledAt, long sampleCycle) {
            this.observed = observed;
            this.sampledAt = sampledAt;
            this.sampleCycle = sampleCycle;
            firstPositionOccupied = first;
            secondPositionOccupied = second;
            thirdPositionOccupied = third;
            conditionedOccupiedPositionCount = (first ? 1 : 0) + (second ? 1 : 0) + (third ? 1 : 0);
            if (third && !second) {
                orderIssue = OrderIssue.THIRD_WITHOUT_SECOND;
            } else if (second && !first) {
                orderIssue = OrderIssue.SECOND_WITHOUT_FIRST;
            } else {
                orderIssue = OrderIssue.NONE;
            }
            full = observed && first && second && third && orderIssue == OrderIssue.NONE;
        }
    }

    private static final Status NOT_OBSERVED = new Status(false, false, false, false,
            LoopTimestamp.unavailable(), -1);

    private final BooleanSource firstOccupied;
    private final BooleanSource secondOccupied;
    private final BooleanSource thirdOccupied;
    private Status status = NOT_OBSERVED;
    private LoopClock ownerClock;
    private boolean updating;
    private boolean stopped;
    private long lifecycleGeneration;
    private final BooleanSource fullSource = clock -> status.full;

    /**
     * Constructs the example owner after validating all configuration and resolving all devices.
     * Construction configures inputs but performs no sensor read.
     */
    public ReferenceInventoryStatusService(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config required = Objects.requireNonNull(config, "config is required");
        String firstName = requireName(required.firstPositionSensorName, "firstPositionSensorName");
        String secondName = requireName(required.secondPositionSensorName, "secondPositionSensorName");
        String thirdName = requireName(required.thirdPositionSensorName, "thirdPositionSensorName");
        double occupiedDelay = requireDelay(required.occupiedDebounceSec, "occupiedDebounceSec");
        double vacatedDelay = requireDelay(required.vacatedDebounceSec, "vacatedDebounceSec");
        requireDistinctNames(firstName, secondName, thirdName);

        DigitalChannel firstChannel = map.get(DigitalChannel.class, firstName);
        DigitalChannel secondChannel = map.get(DigitalChannel.class, secondName);
        DigitalChannel thirdChannel = map.get(DigitalChannel.class, thirdName);
        requireDistinctDevices(firstChannel, secondChannel, thirdChannel);

        firstOccupied = FtcSensors.digitalLow(firstChannel)
                .debouncedOnOff(occupiedDelay, vacatedDelay);
        secondOccupied = FtcSensors.digitalLow(secondChannel)
                .debouncedOnOff(occupiedDelay, vacatedDelay);
        thirdOccupied = FtcSensors.digitalLow(thirdChannel)
                .debouncedOnOff(occupiedDelay, vacatedDelay);
    }

    /** Returns the most recently published immutable snapshot without sampling hardware. */
    public Status status() {
        return status;
    }

    /** Returns one stable source identity projecting {@link Status#full} from the cached status. */
    public BooleanSource fullSource() {
        return fullSource;
    }

    @Override
    public void start(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        if (updating) {
            throw new IllegalStateException("Reference inventory cannot start during sampling.");
        }
        lifecycleGeneration++;
        stopped = false;
        resetSources();
        status = NOT_OBSERVED;
        update(clock);
    }

    /** Publishes once on complete successful sampling; failed reads do not consume the cycle. */
    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");
        if (stopped) {
            return;
        }
        if (updating) {
            throw new IllegalStateException("Reference inventory sampling must not be reentrant.");
        }
        LoopTimestamp sampledAt = clock.nowTimestamp();
        if (ownerClock != null && ownerClock != clock) {
            throw new IllegalArgumentException("Reference inventory requires its one owner LoopClock.");
        }
        ownerClock = clock;
        long sampledCycle = clock.cycle();
        if (status.observed && status.sampleCycle == sampledCycle) {
            return;
        }
        long generation = lifecycleGeneration;
        updating = true;
        try {
            boolean first = firstOccupied.getAsBoolean(clock);
            if (generation != lifecycleGeneration) {
                return;
            }
            boolean second = secondOccupied.getAsBoolean(clock);
            if (generation != lifecycleGeneration) {
                return;
            }
            boolean third = thirdOccupied.getAsBoolean(clock);
            if (generation != lifecycleGeneration) {
                return;
            }
            if (sampledCycle != clock.cycle()) {
                throw new IllegalStateException("Do not advance the LoopClock during inventory sampling.");
            }
            status = new Status(true, first, second, third, sampledAt, sampledCycle);
        } finally {
            updating = false;
            // A sensor callback may stop the owner, but its source operation must unwind before
            // resetting that source. No in-flight read may republish after the generation changes.
            if (stopped) {
                resetSources();
            }
        }
    }

    /** Withdraws all sample evidence; only an explicit later service start can sample again. */
    @Override
    public void stop() {
        lifecycleGeneration++;
        stopped = true;
        status = NOT_OBSERVED;
        if (!updating) {
            resetSources();
        }
    }

    private void resetSources() {
        firstOccupied.reset();
        secondOccupied.reset();
        thirdOccupied.reset();
    }

    private static String requireName(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must not be blank");
        }
        return value.trim();
    }

    private static double requireDelay(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(field + " must be finite and >= 0, got " + value);
        }
        return value;
    }

    private static void requireDistinctNames(String first, String second, String third) {
        if (first.equals(second) || first.equals(third) || second.equals(third)) {
            throw new IllegalArgumentException("inventory sensor names must be pairwise distinct");
        }
    }

    private static void requireDistinctDevices(
            DigitalChannel first, DigitalChannel second, DigitalChannel third) {
        if (first == second || first == third || second == third) {
            throw new IllegalArgumentException("inventory sensor names must resolve to distinct devices");
        }
    }
}
