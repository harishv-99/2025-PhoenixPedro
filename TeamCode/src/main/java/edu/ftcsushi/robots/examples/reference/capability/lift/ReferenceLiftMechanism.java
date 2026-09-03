package edu.ftcsushi.robots.examples.reference.capability.lift;

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

/** Owns a bounded lift Plant and exposes reference-aware semantic commands. */
public final class ReferenceLiftMechanism implements ReferenceLift, RobotProgram.Output {
    /** Data-only lift wiring, coordinate, and homing recipe. */
    public static final class Config {
        public String motorName;
        public Direction direction;
        public String bottomSwitchName;
        public double maximumHeightIn;
        public double ticksPerIn;
        public double toleranceIn;
        public double maximumPower;
        public double stowedHeightIn;
        public double lowHeightIn;
        public double highHeightIn;
        public double homingPower;
        public double homingTimeoutSec;
        public double moveTimeoutSec;

        private Config() {
        }

        /** Returns compiling placeholders that require physical review before use. */
        public static Config defaults() {
            Config c = new Config();
            c.motorName = "liftMotor";
            c.direction = Direction.FORWARD;
            c.bottomSwitchName = "liftBottom";
            c.maximumHeightIn = 18.0;
            c.ticksPerIn = 100.0;
            c.toleranceIn = 0.20;
            c.maximumPower = 0.30;
            c.stowedHeightIn = 0.0;
            c.lowHeightIn = 4.0;
            c.highHeightIn = 14.0;
            c.homingPower = -0.15;
            c.homingTimeoutSec = 3.0;
            c.moveTimeoutSec = 2.0;
            return c;
        }
    }

    private final PositionPlant lift;
    private final BooleanSource bottomSwitch;
    private final double homingPower;
    private final double homingTimeoutSec;
    private final double moveTimeoutSec;
    private final SemanticScalarCommand<Height> heightCommand;

    /** Constructs the lift after validating every retained configuration field. */
    public ReferenceLiftMechanism(HardwareMap hardwareMap, Config config) {
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

        bottomSwitch = FtcSensors.digitalLow(map, c.bottomSwitchName)
                .debouncedOnOff(0.02, 0.02);
        lift = FtcActuators.plant(map)
                .motor(c.motorName, c.direction)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(0.0, c.maximumHeightIn)
                .scaleToNative(c.ticksPerIn)
                .needsReference("reference lift has not been homed")
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

    @Override
    public void update(LoopClock clock) {
        lift.update(clock);
    }

    @Override
    public void stop() {
        lift.stop();
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "ReferenceLiftMechanism.Config is required");
        Config c = new Config();
        c.motorName = name(s.motorName, "motorName");
        c.direction = Objects.requireNonNull(s.direction, "direction");
        c.bottomSwitchName = name(s.bottomSwitchName, "bottomSwitchName");
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
                    "semantic lift heights must satisfy stowedHeightIn < lowHeightIn "
                            + "< highHeightIn <= maximumHeightIn, got "
                            + c.stowedHeightIn + ", " + c.lowHeightIn + ", "
                            + c.highHeightIn + ", " + c.maximumHeightIn);
        }
        c.homingPower = s.homingPower;
        if (!Double.isFinite(c.homingPower) || c.homingPower >= 0.0 || c.homingPower < -1.0) {
            throw new IllegalArgumentException("homingPower must be finite and in [-1, 0)");
        }
        c.homingTimeoutSec = positive(s.homingTimeoutSec, "homingTimeoutSec");
        c.moveTimeoutSec = positive(s.moveTimeoutSec, "moveTimeoutSec");
        return c;
    }

    private static String name(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must be a non-blank FTC hardware name");
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
