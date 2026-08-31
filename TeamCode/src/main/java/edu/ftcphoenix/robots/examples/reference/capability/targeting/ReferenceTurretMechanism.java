package edu.ftcphoenix.robots.examples.reference.capability.targeting;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.PlantTargetRequest;
import edu.ftcphoenix.fw.actuation.PlantTargetResolution;
import edu.ftcphoenix.fw.actuation.PlantTargetResolver;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.RobotProgram;

/**
 * Optional Reference example that realizes one coordinated turret request through a bounded Plant.
 *
 * <p>The mechanism reads only the cached solution published by
 * {@link ReferenceCoordinatedShotService}. It neither samples nor resets localization, and it owns
 * the turret motor, final planner, Plant heartbeat, and stop lifecycle. Planner selection or a
 * fallback resolution is a requested-target fact, not proof that the physical turret arrived.</p>
 */
public final class ReferenceTurretMechanism implements RobotProgram.Output {

    /** Data-only turret wiring, radian coordinate, and finite cable-bound recipe. */
    public static final class Config {
        /** FTC configuration name for the turret motor. */
        public String motorName;
        /** Logical motor direction for increasing turret angle. */
        public Direction direction;
        /** Inclusive minimum legal turret angle, in radians. */
        public double minimumAngleRad;
        /** Inclusive maximum legal turret angle, in radians. */
        public double maximumAngleRad;
        /** Native encoder ticks per turret radian. */
        public double ticksPerRad;
        /** Plant completion tolerance, in radians. */
        public double positionToleranceRad;
        /** Hold target used only when an unavailable entry has no finite measurement, in radians. */
        public double initialHoldAngleRad;

        private Config() {
        }

        /**
         * Returns compiling software values, not reviewed cable bounds, encoder zero, or tuning.
         */
        public static Config defaults() {
            Config c = new Config();
            c.motorName = "turretMotor";
            c.direction = Direction.FORWARD;
            c.minimumAngleRad = -1.5 * Math.PI;
            c.maximumAngleRad = 1.5 * Math.PI;
            c.ticksPerRad = 1000.0;
            c.positionToleranceRad = Math.toRadians(1.0);
            c.initialHoldAngleRad = 0.0;
            return c;
        }
    }

    private static final double FULL_TURN_RAD = 2.0 * Math.PI;

    private final PositionPlant turret;

    /**
     * Constructs the complete turret realization after defensively validating its configuration.
     *
     * <p>{@code alreadyReferenced()} below is an explicit example assumption: an adopting robot
     * must establish that native encoder zero really matches its turret-radian zero before using
     * this recipe. The configured bounds likewise require a physical cable and collision review.</p>
     *
     * @param hardwareMap FTC hardware registry that owns the configured motor
     * @param config data-only motor, coordinate, bounds, and fallback configuration
     * @param shotService upstream owner of the cached coordinated solution
     */
    public ReferenceTurretMechanism(HardwareMap hardwareMap,
                                    Config config,
                                    ReferenceCoordinatedShotService shotService) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);
        ReferenceCoordinatedShotService service = Objects.requireNonNull(
                shotService,
                "shotService is required"
        );

        // This projection deliberately has no owned state and therefore no reset propagation.
        // The service lifecycle publishes the immutable snapshot before the output phase reads it.
        Source<PlantTargetRequest> requestSource = Source.of(
                clock -> service.solution().turretRequest
        );
        PlantTargetResolver finalTarget = PlantTargets.plan(requestSource)
                .nearestToMeasurement()
                .rejectUnreachable()
                .accept()
                .maxObservationAgeSec(service.plannerObservationMaxAgeSec())
                .minQuality(service.plannerMinimumObservationQuality())
                .doneAccept()
                .whenUnavailable()
                .holdMeasuredTargetOnEntry(c.initialHoldAngleRad);

        turret = FtcActuators.plant(map)
                .motor(c.motorName, c.direction)
                .position()
                .deviceManaged()
                .periodic(FULL_TURN_RAD)
                .bounded(c.minimumAngleRad, c.maximumAngleRad)
                .scaleToNative(c.ticksPerRad)
                .alreadyReferenced()
                .positionTolerance(c.positionToleranceRad)
                .targetFromResolver(finalTarget)
                .build();
    }

    /**
     * Returns the latest cached planner/Plant target-selection result.
     *
     * <p>This result distinguishes selected intent from hold behavior. It does not claim physical
     * readiness or arrival.</p>
     */
    public PlantTargetResolution targetResolution() {
        return turret.getTargetResolution();
    }

    /** Advances the one privately owned turret Plant in the managed output phase. */
    @Override
    public void update(LoopClock clock) {
        turret.update(clock);
    }

    /** Terminally stops the privately owned turret Plant; repeated calls are harmless. */
    @Override
    public void stop() {
        turret.stop();
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(
                source,
                "ReferenceTurretMechanism.Config is required"
        );
        Config c = new Config();
        c.motorName = name(s.motorName, "motorName");
        c.direction = Objects.requireNonNull(s.direction, "direction");
        c.minimumAngleRad = finite(s.minimumAngleRad, "minimumAngleRad");
        c.maximumAngleRad = finite(s.maximumAngleRad, "maximumAngleRad");
        if (c.minimumAngleRad >= c.maximumAngleRad) {
            throw new IllegalArgumentException(
                    "turret bounds must satisfy minimumAngleRad < maximumAngleRad, got "
                            + c.minimumAngleRad + " and " + c.maximumAngleRad
            );
        }
        c.ticksPerRad = positive(s.ticksPerRad, "ticksPerRad");
        c.positionToleranceRad = nonnegative(
                s.positionToleranceRad,
                "positionToleranceRad"
        );
        c.initialHoldAngleRad = finite(s.initialHoldAngleRad, "initialHoldAngleRad");
        if (c.initialHoldAngleRad < c.minimumAngleRad
                || c.initialHoldAngleRad > c.maximumAngleRad) {
            throw new IllegalArgumentException(
                    "initialHoldAngleRad must be in [minimumAngleRad, maximumAngleRad] = ["
                            + c.minimumAngleRad + ", " + c.maximumAngleRad + "], got "
                            + c.initialHoldAngleRad
            );
        }
        return c;
    }

    private static String name(String value, String field) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(field + " must be a non-blank FTC hardware name");
        }
        return value.trim();
    }

    private static double finite(double value, String field) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(field + " must be finite, got " + value);
        }
        return value;
    }

    private static double positive(double value, String field) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(field + " must be finite and > 0, got " + value);
        }
        return value;
    }

    private static double nonnegative(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(field + " must be finite and >= 0, got " + value);
        }
        return value;
    }
}
