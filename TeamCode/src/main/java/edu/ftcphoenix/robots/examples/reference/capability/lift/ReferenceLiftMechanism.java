package edu.ftcphoenix.robots.examples.reference.capability.lift;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.PositionCalibrationTasks;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcSensors;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.task.Task;

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
            return c;
        }
    }

    private final PositionPlant lift;
    private final BooleanSource bottomSwitch;
    private final double stowedHeightIn;
    private final double lowHeightIn;
    private final double highHeightIn;
    private final double homingPower;
    private final double homingTimeoutSec;
    private Height requestedHeight = Height.STOWED;
    private Status lastStatus = new Status(Height.STOWED, 0.0, Double.NaN, false, false);

    /** Constructs the lift after validating every retained configuration field. */
    public ReferenceLiftMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);
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
                .targetFromNewCommand(c.stowedHeightIn)
                .build();
        stowedHeightIn = c.stowedHeightIn;
        lowHeightIn = c.lowHeightIn;
        highHeightIn = c.highHeightIn;
        homingPower = c.homingPower;
        homingTimeoutSec = c.homingTimeoutSec;
    }

    @Override
    public void setHeight(Height height) {
        requestedHeight = Objects.requireNonNull(height, "height");
        lift.commandTarget().set(positionFor(height));
    }

    @Override
    public Task home() {
        return PositionCalibrationTasks.search(lift)
                .withPower(homingPower)
                .until(bottomSwitch)
                .establishReferenceAt(0.0)
                .holdAfterReference(stowedHeightIn)
                .failAfterSec(homingTimeoutSec)
                .build();
    }

    @Override
    public Status status() {
        return lastStatus;
    }

    @Override
    public void update(LoopClock clock) {
        lift.update(clock);
        lastStatus = new Status(
                requestedHeight,
                lift.commandTarget().get(),
                lift.getMeasurement(),
                lift.isReferenced(),
                lift.atTarget());
    }

    @Override
    public void stop() {
        lift.stop();
    }

    private double positionFor(Height height) {
        switch (height) {
            case HIGH:
                return highHeightIn;
            case LOW:
                return lowHeightIn;
            case STOWED:
            default:
                return stowedHeightIn;
        }
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
        c.homingPower = s.homingPower;
        if (!Double.isFinite(c.homingPower) || c.homingPower >= 0.0 || c.homingPower < -1.0) {
            throw new IllegalArgumentException("homingPower must be finite and in [-1, 0)");
        }
        c.homingTimeoutSec = positive(s.homingTimeoutSec, "homingTimeoutSec");
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
