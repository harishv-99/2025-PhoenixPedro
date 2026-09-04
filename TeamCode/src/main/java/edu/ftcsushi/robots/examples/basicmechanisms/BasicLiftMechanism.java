package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PositionCalibrationTasks;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.SemanticScalarCommand;
import edu.ftcsushi.fw.actuation.SemanticScalarTasks;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Owns one bounded referenced-position Plant and realizes {@link BasicLift} intent. */
public final class BasicLiftMechanism implements BasicLift, RobotProgram.Output {

    /** Data-only lift wiring, coordinate, limits, and timeout choices. */
    public static final class Config {
        /** Nonblank FTC Robot Configuration name for the lift motor. */
        public String motorName;

        /** Logical motor direction; verify that positive position moves away from the bottom. */
        public Direction direction;

        /**
         * Nonblank FTC digital-channel name for the bottom switch. The electrical input is
         * active-low: LOW means pressed/at the bottom reference, while HIGH means released.
         */
        public String bottomSwitchName;

        /** Maximum legal lift target in mechanism inches; finite and greater than zero. */
        public double maximumHeightIn;

        /** Positive encoder conversion in native ticks per mechanism inch. */
        public double ticksPerIn;

        /**
         * Symmetric feedback completion tolerance in mechanism inches; finite, positive, and
         * smaller than half the closest adjacent named-height gap.
         */
        public double toleranceIn;

        /** Maximum device-managed motor-power magnitude in range {@code (0, 1]}. */
        public double maximumPower;

        /** Named {@link Height#STOWED} position in mechanism inches. */
        public double stowedHeightIn;

        /** Named {@link Height#LOW} position in mechanism inches. */
        public double lowHeightIn;

        /** Named {@link Height#HIGH} position in mechanism inches. */
        public double highHeightIn;

        /**
         * Normalized search power toward the bottom switch, finite in {@code [-1, 0)}. This
         * polarity is a software convention that must be verified on the physical mechanism.
         */
        public double homingPower;

        /** Positive homing-search timeout in seconds. */
        public double homingTimeoutSec;

        /** Positive feedback-move timeout in seconds. */
        public double moveTimeoutSec;

        private Config() {
            // Start from defaults() so the whole software recipe is present.
        }

        /** Returns a complete software baseline, not reviewed physical facts. */
        public static Config defaults() {
            Config config = new Config();
            config.motorName = "liftMotor";
            config.direction = Direction.FORWARD;
            config.bottomSwitchName = "liftBottom";
            config.maximumHeightIn = 18.0;
            config.ticksPerIn = 100.0;
            config.toleranceIn = 0.20;
            config.maximumPower = 0.30;
            config.stowedHeightIn = 0.0;
            config.lowHeightIn = 4.0;
            config.highHeightIn = 14.0;
            config.homingPower = -0.15;
            config.homingTimeoutSec = 3.0;
            config.moveTimeoutSec = 2.0;
            return config;
        }
    }

    private final PositionPlant lift;
    private final BooleanSource bottomSwitch;
    private final double homingPower;
    private final double homingTimeoutSec;
    private final double moveTimeoutSec;
    private final SemanticScalarCommand<Height> heightCommand;

    /**
     * Validates a defensive configuration snapshot, then constructs and privately owns the Plant.
     *
     * @param hardwareMap FTC hardware registry
     * @param config complete data-only lift configuration
     */
    public BasicLiftMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        homingPower = c.homingPower;
        homingTimeoutSec = c.homingTimeoutSec;
        moveTimeoutSec = c.moveTimeoutSec;
        heightCommand = SemanticScalarCommand.forEnum(Height.STOWED)
                .map(Height.STOWED, c.stowedHeightIn)
                .map(Height.LOW, c.lowHeightIn)
                .map(Height.HIGH, c.highHeightIn)
                .build();

        bottomSwitch = FtcSensors.digitalLow(
                map, c.bottomSwitchName)
                .debouncedOnOff(0.02, 0.02);
        lift = FtcActuators.plant(map)
                .motor(c.motorName, c.direction)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(0.0, c.maximumHeightIn)
                .scaleToNative(c.ticksPerIn)
                .needsReference("basic lift has not been homed")
                .positionTolerance(c.toleranceIn)
                .outputPowerLimitedTo(c.maximumPower)
                .targetExactlyFrom(heightCommand)
                .build();
    }

    @Override
    public void setHeight(Height height) {
        heightCommand.set(Objects.requireNonNull(height, "height"));
    }

    @Override
    public Task moveTo(Height height) {
        return SemanticScalarTasks.set(heightCommand, Objects.requireNonNull(height, "height"))
                .untilReachedBy(lift)
                .leaveRequestOnCancel()
                .timeout(moveTimeoutSec)
                .build();
    }

    @Override
    public Task home() {
        Task search = PositionCalibrationTasks.search(lift)
                .withPower(homingPower)
                .until(bottomSwitch)
                .establishReferenceAt(0.0)
                .failAfterSec(homingTimeoutSec)
                .build();

        return Tasks.sequence(
                search,
                SemanticScalarTasks.set(heightCommand, Height.STOWED).build());
    }

    @Override
    public Status status() {
        return new Status(heightCommand.snapshot(lift.snapshot()));
    }

    /** Applies the current request once and refreshes the Plant's cached evidence. */
    @Override
    public void update(LoopClock clock) {
        lift.update(clock);
    }

    /** Terminally stops the one privately owned Plant. */
    @Override
    public void stop() {
        lift.stop();
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "BasicLiftMechanism.Config is required");
        Config c = new Config();
        c.motorName = hardwareName(s.motorName, "motorName");
        c.direction = Objects.requireNonNull(s.direction, "direction is required");
        c.bottomSwitchName = hardwareName(s.bottomSwitchName, "bottomSwitchName");
        c.maximumHeightIn = positive(s.maximumHeightIn, "maximumHeightIn");
        c.ticksPerIn = positive(s.ticksPerIn, "ticksPerIn");
        c.toleranceIn = positive(s.toleranceIn, "toleranceIn");
        c.maximumPower = positive(s.maximumPower, "maximumPower");
        if (c.maximumPower > 1.0) {
            throw new IllegalArgumentException("maximumPower must be <= 1.0");
        }
        c.stowedHeightIn = within(s.stowedHeightIn, c.maximumHeightIn, "stowedHeightIn");
        c.lowHeightIn = within(s.lowHeightIn, c.maximumHeightIn, "lowHeightIn");
        c.highHeightIn = within(s.highHeightIn, c.maximumHeightIn, "highHeightIn");
        if (!(c.stowedHeightIn < c.lowHeightIn && c.lowHeightIn < c.highHeightIn)) {
            throw new IllegalArgumentException(
                    "lift heights must satisfy stowedHeightIn < lowHeightIn < highHeightIn "
                            + "<= maximumHeightIn, got " + c.stowedHeightIn + ", "
                            + c.lowHeightIn + ", " + c.highHeightIn + ", "
                            + c.maximumHeightIn);
        }
        double closestNamedHeightGap = Math.min(
                c.lowHeightIn - c.stowedHeightIn,
                c.highHeightIn - c.lowHeightIn);
        double largestNonoverlappingTolerance = closestNamedHeightGap / 2.0;
        if (c.toleranceIn >= largestNonoverlappingTolerance) {
            throw new IllegalArgumentException(
                    "toleranceIn must be strictly less than half the closest adjacent "
                            + "named-height gap so their arrival bands cannot overlap; got "
                            + "tolerance " + c.toleranceIn + " and half-gap "
                            + largestNonoverlappingTolerance);
        }
        c.homingPower = s.homingPower;
        if (!Double.isFinite(c.homingPower) || c.homingPower < -1.0 || c.homingPower >= 0.0) {
            throw new IllegalArgumentException("homingPower must be finite and in [-1, 0)");
        }
        c.homingTimeoutSec = positive(s.homingTimeoutSec, "homingTimeoutSec");
        c.moveTimeoutSec = positive(s.moveTimeoutSec, "moveTimeoutSec");
        return c;
    }

    private static String hardwareName(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "BasicLiftMechanism.Config." + field
                            + " must be a non-blank FTC hardware name");
        }
        return value;
    }

    private static double positive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double within(double value, double maximum, String field) {
        if (!Double.isFinite(value) || value < 0.0 || value > maximum) {
            throw new IllegalArgumentException(
                    field + " must be finite and in [0, " + maximum + "], got " + value);
        }
        return value;
    }
}
