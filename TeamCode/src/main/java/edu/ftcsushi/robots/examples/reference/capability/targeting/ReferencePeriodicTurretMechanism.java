package edu.ftcsushi.robots.examples.reference.capability.targeting;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PlantTargetResolver;
import edu.ftcsushi.fw.actuation.PlantTargets;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.PositionPlantSnapshot;
import edu.ftcsushi.fw.actuation.ScalarTasks;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/**
 * Focused numeric turret whose periodic command selects the nearest legal physical representative.
 *
 * <p>A request such as {@code 0.1 rad} can be realized at {@code 0.1 + k * 2π rad}. The
 * mechanism owns that selection, the bounded motor Plant, its one heartbeat, and terminal stop.
 * The separate {@link ReferenceTurretMechanism} remains the advanced example for selecting among
 * timestamped coordinated-shot requests.</p>
 */
public final class ReferencePeriodicTurretMechanism implements RobotProgram.Output {

    /** Data-only motor wiring and radian-coordinate facts. */
    public static final class Config {
        /** FTC configuration name for the turret motor. */
        public String motorName;
        /** Logical motor direction for increasing turret angle. */
        public Direction direction;
        /** Inclusive minimum legal physical target, in radians. */
        public double minimumAngleRad;
        /** Inclusive maximum legal physical target, in radians. */
        public double maximumAngleRad;
        /** Native encoder ticks per turret radian. */
        public double ticksPerRad;
        /** Inclusive Plant arrival tolerance, in radians. */
        public double positionToleranceRad;
        /** Initial logical angle and unavailable-entry fallback, in radians. */
        public double initialAngleRad;

        private Config() {
        }

        /** Returns compiling software values, not reviewed cable bounds, zero, or tuning. */
        public static Config defaults() {
            Config c = new Config();
            c.motorName = "periodicTurret";
            c.direction = Direction.FORWARD;
            c.minimumAngleRad = -3.0 * Math.PI;
            c.maximumAngleRad = 3.0 * Math.PI;
            c.ticksPerRad = 1000.0;
            c.positionToleranceRad = Math.toRadians(1.0);
            c.initialAngleRad = 0.0;
            return c;
        }
    }

    /** Immutable navigation from logical request through physical selection to arrival evidence. */
    public static final class Status {
        private final PositionPlantSnapshot plantSnapshot;

        private Status(PositionPlantSnapshot plantSnapshot) {
            this.plantSnapshot = Objects.requireNonNull(plantSnapshot, "plantSnapshot");
        }

        /** Returns the logical periodic command, in radians. */
        public double requestedAngleRad() {
            return plantSnapshot.commandTarget();
        }

        /**
         * Returns the physical representative selected by the resolver, in radians.
         *
         * <p>Inspect {@link #requestWasSelected()} before treating this as a representative of the
         * active request; an unavailable request can instead select the documented hold target.</p>
         */
        public double selectedAngleRad() {
            return plantSnapshot.requestedTarget();
        }

        /** Returns the final post-bounds/guards motor target, in radians. */
        public double appliedAngleRad() {
            return plantSnapshot.appliedTarget();
        }

        /** Returns the captured encoder-derived measurement, in radians. */
        public double measuredAngleRad() {
            return plantSnapshot.measurement();
        }

        /** Returns whether the selected target truthfully satisfies the logical periodic request. */
        public boolean requestWasSelected() {
            return plantSnapshot.targetResolution().satisfiesIntent();
        }

        /** Returns command-correlated physical arrival at the selected representative. */
        public boolean arrived() {
            return plantSnapshot.atCommandTarget();
        }

        /** Returns the complete immutable position-Plant capture for advanced diagnostics. */
        public PositionPlantSnapshot plantSnapshot() {
            return plantSnapshot;
        }
    }

    private static final double FULL_TURN_RAD = 2.0 * Math.PI;

    // This target has an independent composed-graph role: the resolver turns its logical value
    // into one bounded physical representative before the Plant writes the motor.
    private final ScalarTarget angleCommand;
    private final PositionPlant turret;

    private Status lastStatus;

    /**
     * Constructs the complete periodic realization after validating all data before lookup.
     *
     * <p>{@code alreadyReferenced()} is an explicit teaching assumption. An adopting robot must
     * establish encoder zero, scale, directions, cable bounds, and collision clearance on the
     * isolated mechanism before enabling motion.</p>
     */
    public ReferencePeriodicTurretMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        angleCommand = ScalarTarget.create(c.initialAngleRad);
        PlantTargetResolver nearestEquivalent = PlantTargets
                .equivalentPositionsOf(angleCommand)
                .nearestToMeasurement()
                .whenUnavailable()
                .holdMeasuredTargetOnEntry(c.initialAngleRad);

        turret = FtcActuators.plant(map)
                .motor(c.motorName, c.direction)
                .position()
                .deviceManaged()
                .periodic(FULL_TURN_RAD)
                .bounded(c.minimumAngleRad, c.maximumAngleRad)
                .scaleToNative(c.ticksPerRad)
                .alreadyReferenced()
                .positionTolerance(c.positionToleranceRad)
                .targetFromResolver(nearestEquivalent)
                .build();
        lastStatus = new Status(turret.snapshot());
    }

    /** Replaces the persistent logical periodic command; the next output update selects it. */
    public void setAngleRad(double angleRad) {
        requireFinite(angleRad, "angleRad");
        angleCommand.set(angleRad);
    }

    /**
     * Builds a fresh feedback-aware Task for one logical angle request.
     *
     * <p>Success means the request selected a legal equivalent and encoder feedback reached that
     * physical representative. Timeout and active cancellation deliberately leave the persistent
     * position request unchanged so the position controller continues holding it; a caller that
     * wants another hold point must request that point explicitly.</p>
     */
    public Task setAngleTask(double angleRad, double timeoutSec) {
        requireFinite(angleRad, "angleRad");
        requirePositive(timeoutSec, "timeoutSec");
        return ScalarTasks.set(angleCommand, angleRad)
                .untilReachedBy(turret)
                .leaveRequestOnCancel()
                .timeout(timeoutSec)
                .build();
    }

    /** Returns the latest immutable requested/selected/applied/measured/arrival publication. */
    public Status status() {
        return lastStatus;
    }

    /** Advances the one privately owned Plant and then replaces the complete cached Status. */
    @Override
    public void update(LoopClock clock) {
        turret.update(clock);
        lastStatus = new Status(turret.snapshot());
    }

    /** Terminally stops the privately owned Plant; repeated calls are harmless. */
    @Override
    public void stop() {
        turret.stop();
        lastStatus = new Status(turret.snapshot());
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(
                source,
                "ReferencePeriodicTurretMechanism.Config is required");
        Config c = new Config();
        c.motorName = requireName(s.motorName, "motorName");
        c.direction = Objects.requireNonNull(s.direction, "direction");
        c.minimumAngleRad = requireFinite(s.minimumAngleRad, "minimumAngleRad");
        c.maximumAngleRad = requireFinite(s.maximumAngleRad, "maximumAngleRad");
        if (c.minimumAngleRad >= c.maximumAngleRad) {
            throw new IllegalArgumentException(
                    "turret bounds must satisfy minimumAngleRad < maximumAngleRad, got "
                            + c.minimumAngleRad + " and " + c.maximumAngleRad);
        }
        c.ticksPerRad = requirePositive(s.ticksPerRad, "ticksPerRad");
        c.positionToleranceRad = requireNonnegative(
                s.positionToleranceRad,
                "positionToleranceRad");
        c.initialAngleRad = requireFinite(s.initialAngleRad, "initialAngleRad");
        if (c.initialAngleRad < c.minimumAngleRad
                || c.initialAngleRad > c.maximumAngleRad) {
            throw new IllegalArgumentException(
                    "initialAngleRad is also the no-measurement hold target and must be in "
                            + "[minimumAngleRad, maximumAngleRad], got " + c.initialAngleRad);
        }
        return c;
    }

    private static String requireName(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must be a non-blank FTC hardware name");
        }
        return value.trim();
    }

    private static double requireFinite(double value, String field) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(field + " must be finite, got " + value);
        }
        return value;
    }

    private static double requirePositive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double requireNonnegative(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(field + " must be finite and >= 0, got " + value);
        }
        return value;
    }
}
