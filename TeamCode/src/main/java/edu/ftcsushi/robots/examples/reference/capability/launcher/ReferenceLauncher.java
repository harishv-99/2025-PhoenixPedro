package edu.ftcsushi.robots.examples.reference.capability.launcher;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.task.Task;

/** Season-neutral velocity-wheel, transfer, and release capability shared by TeleOp and Auto. */
public interface ReferenceLauncher {
    /**
     * Immutable grouped-flywheel and per-wheel operational evidence.
     *
     * <p>{@link #leftAtTarget()} and {@link #rightAtTarget()} are pure finite,
     * inclusive-tolerance facts, including at a zero request. {@link #ready()} additionally
     * requires a positive request value which the active Plant intent selected and the Plant
     * applied without fallback, planner clamp, or hardware-target modification. It deliberately
     * does not use the grouped aggregate arrival sample in place of the later independent wheel
     * samples. Controller readiness is not proof that an object launched or scored.</p>
     */
    final class Status {
        private final PlantSnapshot flywheelSnapshot;
        private final double leftMeasuredVelocityTicksPerSec;
        private final double rightMeasuredVelocityTicksPerSec;
        private final double velocityToleranceTicksPerSec;
        private final boolean objectPresent;
        private final boolean transferPulseActive;

        /**
         * Composes one grouped Plant capture with application-owned per-wheel and sensor evidence.
         *
         * @param flywheelSnapshot grouped flywheel Plant capture from the same owner publication
         * @param leftMeasuredVelocityTicksPerSec measured left native encoder ticks per second
         * @param rightMeasuredVelocityTicksPerSec measured right native encoder ticks per second
         * @param velocityToleranceTicksPerSec finite inclusive per-wheel tolerance, at least zero
         * @param objectPresent conditioned semantic object-sensor state
         * @param transferPulseActive whether the temporary transfer override is active
         * @throws NullPointerException if {@code flywheelSnapshot} is null
         * @throws IllegalArgumentException if the snapshot lacks a finite command target, or if
         *                                  {@code velocityToleranceTicksPerSec} is non-finite or
         *                                  negative
         */
        public Status(PlantSnapshot flywheelSnapshot,
                      double leftMeasuredVelocityTicksPerSec,
                      double rightMeasuredVelocityTicksPerSec,
                      double velocityToleranceTicksPerSec,
                      boolean objectPresent,
                      boolean transferPulseActive) {
            this.flywheelSnapshot = Objects.requireNonNull(
                    flywheelSnapshot,
                    "flywheelSnapshot");
            if (!flywheelSnapshot.hasCommandTarget()
                    || !Double.isFinite(flywheelSnapshot.commandTarget())) {
                throw new IllegalArgumentException(
                        "flywheelSnapshot must carry a finite command target");
            }
            if (!Double.isFinite(velocityToleranceTicksPerSec)
                    || velocityToleranceTicksPerSec < 0.0) {
                throw new IllegalArgumentException(
                        "velocityToleranceTicksPerSec must be finite and >= 0, got "
                                + velocityToleranceTicksPerSec);
            }
            this.leftMeasuredVelocityTicksPerSec = leftMeasuredVelocityTicksPerSec;
            this.rightMeasuredVelocityTicksPerSec = rightMeasuredVelocityTicksPerSec;
            this.velocityToleranceTicksPerSec = velocityToleranceTicksPerSec;
            this.objectPresent = objectPresent;
            this.transferPulseActive = transferPulseActive;
        }

        /** Returns the captured persistent flywheel request in native encoder ticks per second. */
        public double requestedVelocityTicksPerSec() {
            return flywheelSnapshot.commandTarget();
        }

        /** Returns the captured final applied velocity in native encoder ticks per second. */
        public double appliedVelocityTicksPerSec() {
            return flywheelSnapshot.appliedTarget();
        }

        /** Returns the captured left measurement in native encoder ticks per second. */
        public double leftMeasuredVelocityTicksPerSec() {
            return leftMeasuredVelocityTicksPerSec;
        }

        /** Returns the captured right measurement in native encoder ticks per second. */
        public double rightMeasuredVelocityTicksPerSec() {
            return rightMeasuredVelocityTicksPerSec;
        }

        /** Returns whether finite left-wheel evidence meets the inclusive configured tolerance. */
        public boolean leftAtTarget() {
            return withinTolerance(leftMeasuredVelocityTicksPerSec);
        }

        /** Returns whether finite right-wheel evidence meets the inclusive configured tolerance. */
        public boolean rightAtTarget() {
            return withinTolerance(rightMeasuredVelocityTicksPerSec);
        }

        /**
         * Returns whether one positive requested value and both independent wheels prove readiness.
         *
         * <p>This is value-level physical readiness, not completion evidence for one command
         * identity. A same-valued active resolver branch is physically equivalent here. The method
         * intentionally does not use {@link PlantSnapshot#atCommandTarget()}, because that query
         * also bundles the grouped Plant's earlier aggregate-arrival sample.</p>
         */
        public boolean ready() {
            double requestedVelocity = requestedVelocityTicksPerSec();
            PlantTargetResolution targetResolution = flywheelSnapshot.targetResolution();
            return requestedVelocity > 0.0
                    && leftAtTarget()
                    && rightAtTarget()
                    && flywheelSnapshot.hasCommandTarget()
                    && Double.isFinite(requestedVelocity)
                    && targetResolution.hasTarget()
                    && targetResolution.satisfiesIntent()
                    && !targetResolution.usedFallback()
                    && !targetResolution.clampedByPlanner()
                    && flywheelSnapshot.targetStatus().accepted()
                    && Double.compare(
                            requestedVelocity,
                            flywheelSnapshot.requestedTarget()) == 0
                    && Double.compare(
                            requestedVelocity,
                            flywheelSnapshot.appliedTarget()) == 0;
        }

        /** Returns the captured conditioned semantic object-sensor state. */
        public boolean objectPresent() {
            return objectPresent;
        }

        /** Returns whether the captured temporary transfer override was active. */
        public boolean transferPulseActive() {
            return transferPulseActive;
        }

        /** Returns the immutable grouped flywheel capture for advanced diagnostics. */
        public PlantSnapshot flywheelSnapshot() {
            return flywheelSnapshot;
        }

        /** Return whether one finite wheel measurement meets this capture's request and tolerance. */
        private boolean withinTolerance(double measuredVelocityTicksPerSec) {
            double requestedVelocityTicksPerSec = requestedVelocityTicksPerSec();
            return Double.isFinite(measuredVelocityTicksPerSec)
                    && Double.isFinite(requestedVelocityTicksPerSec)
                    && Math.abs(measuredVelocityTicksPerSec - requestedVelocityTicksPerSec)
                    <= velocityToleranceTicksPerSec;
        }
    }

    /**
     * Replaces the held flywheel velocity request in native encoder ticks per second.
     *
     * <p>The value must be finite and inside the mechanism's configured inclusive range. This
     * request does not invalidate an active or queued launch. Zero requests flywheel idle only;
     * use {@link #abortLaunches()} to invalidate launch Tasks and clear their temporary release and
     * transfer requests as well.</p>
     *
     * @param velocityTicksPerSec requested flywheel velocity in native encoder ticks per second
     * @throws IllegalArgumentException if the request is non-finite or outside the configured range
     */
    void setTargetVelocityTicksPerSec(double velocityTicksPerSec);

    /**
     * Invalidates every launch Task created before this call and requests active-match idle.
     *
     * <p>The request clears temporary transfer work, selects the configured retracted release
     * position, and selects zero flywheel velocity. Those source requests are realized by a later
     * normal output update; this method does not terminally stop the mechanism's Plants. A later
     * call to {@link #launchOne()} creates a launch in the new valid generation.</p>
     */
    void abortLaunches();

    /**
     * Builds a fresh, single-use spin-up, release, transfer, and cleanup Task.
     *
     * <p>The Task reports success, spin-up timeout, or cancellation without leaving a wheel,
     * release, or temporary transfer request active.</p>
     */
    Task launchOne();

    /**
     * Returns the latest complete cached publication of grouped Plant facts and launcher-specific
     * per-wheel, object-sensor, transfer, and readiness evidence.
     */
    Status status();
}
