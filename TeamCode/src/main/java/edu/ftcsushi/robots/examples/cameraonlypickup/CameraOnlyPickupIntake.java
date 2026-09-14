package edu.ftcsushi.robots.examples.cameraonlypickup;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.SemanticScalarCommand;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.FtcSensors;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.sensing.observation.OccupancyObservation;

/** Owns one semantic intake request, its private Plant, and an independent occupancy sensor. */
public final class CameraOnlyPickupIntake implements RobotProgram.Output {
    /** Data-only motor/sensor wiring and action command; constructor snapshots every field. */
    public static final class Config {
        /** FTC motor name. */
        public String motorName = "intakeMotor";
        /** Direction making positive collection power feed inward. */
        public Direction direction = Direction.FORWARD;
        /** FTC digital-channel name; reviewed wiring makes LOW mean occupied. */
        public String occupancySwitchName = "intakeOccupied";
        /** Finite nonzero normalized collection power in [-1,1]. */
        public double collectPower = 0.20;

        private Config() { }
        /** Returns a fresh software baseline; it is not proof of wiring or safe intake power. */
        public static Config defaults() { return new Config(); }
    }

    private final Plant plant;
    private final SemanticScalarCommand<Boolean> collecting;
    private final Source<OccupancyObservation> occupancy;
    private boolean stopped;

    /** Validates configuration, acquires the input, then privately constructs the final Plant. */
    public CameraOnlyPickupIntake(HardwareMap hardwareMap, Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(config, "config");
        String motorName = name(config.motorName, "motorName");
        String sensorName = name(config.occupancySwitchName, "occupancySwitchName");
        Direction direction = Objects.requireNonNull(config.direction, "direction");
        double power = config.collectPower;
        if (!Double.isFinite(power) || power == 0 || Math.abs(power) > 1) {
            throw new IllegalArgumentException("collectPower must be finite, nonzero, and in [-1,1]");
        }
        BooleanSource occupied = FtcSensors.digitalLow(hardwareMap, sensorName);
        Source<OccupancyObservation> sampled = Source.<OccupancyObservation>of(clock ->
                OccupancyObservation.observed(occupied.getAsBoolean(clock), clock.nowTimestamp()))
                .memoized();
        occupancy = Source.of(clock -> stopped ? OccupancyObservation.unavailable() : sampled.get(clock));
        collecting = SemanticScalarCommand.create(false, enabled -> enabled ? power : 0.0);
        plant = FtcActuators.plant(hardwareMap).motor(motorName, direction).power()
                .targetExactlyFrom(collecting).build();
    }

    /** Publishes semantic collection intent through the same command used by every client. */
    public void setCollecting(boolean enabled) {
        if (!stopped) collecting.set(enabled);
    }

    /**
     * Borrows one same-cycle memoized sensor read with its actual polling timestamp.
     * This example uses a reviewed stable active-low sensor, with no debounce or command echo.
     */
    public Source<OccupancyObservation> occupancy() { return occupancy; }

    /** Realizes current intent in the program's downstream output phase. */
    @Override public void update(LoopClock clock) { plant.update(clock); }

    /** Terminal physical Plant stop; repeated cleanup is harmless. */
    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        plant.stop();
    }

    /** Rejects missing FTC device names before acquisition. */
    private static String name(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must name an FTC device");
        }
        return value.trim();
    }
}
