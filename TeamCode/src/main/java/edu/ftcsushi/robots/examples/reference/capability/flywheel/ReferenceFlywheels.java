package edu.ftcsushi.robots.examples.reference.capability.flywheel;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.task.Task;

/**
 * Numeric paired-flywheel capability with independent wheel-readiness evidence.
 *
 * <p>The scalar velocity is the complete public request, so direct and deferred paths use the
 * same numeric vocabulary. A grouped Plant owns the one paired command and stop path; this
 * capability adds the two member measurements that a shooter needs before feeding an object.</p>
 */
public interface ReferenceFlywheels {

    /** Immutable grouped-Plant facts plus the two independently sampled wheel measurements. */
    final class Status {
        private final PlantSnapshot plantSnapshot;
        private final double leftMeasuredVelocityTicksPerSec;
        private final double rightMeasuredVelocityTicksPerSec;
        private final double velocityToleranceTicksPerSec;

        /** Package-owned construction prevents callers from fabricating an incoherent capture. */
        Status(PlantSnapshot plantSnapshot,
               double leftMeasuredVelocityTicksPerSec,
               double rightMeasuredVelocityTicksPerSec,
               double velocityToleranceTicksPerSec) {
            this.plantSnapshot = Objects.requireNonNull(plantSnapshot, "plantSnapshot");
            if (!plantSnapshot.hasCommandTarget()
                    || !Double.isFinite(plantSnapshot.commandTarget())) {
                throw new IllegalArgumentException(
                        "plantSnapshot must carry a finite flywheel command target");
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
        }

        /** Returns the captured persistent command, in native encoder ticks per second. */
        public double requestedVelocityTicksPerSec() {
            return plantSnapshot.commandTarget();
        }

        /** Returns the captured resolver-selected grouped target, in ticks per second. */
        public double selectedVelocityTicksPerSec() {
            return plantSnapshot.requestedTarget();
        }

        /** Returns the captured final grouped target, in native encoder ticks per second. */
        public double appliedVelocityTicksPerSec() {
            return plantSnapshot.appliedTarget();
        }

        /** Returns the captured left-wheel measurement, in native encoder ticks per second. */
        public double leftMeasuredVelocityTicksPerSec() {
            return leftMeasuredVelocityTicksPerSec;
        }

        /** Returns the captured right-wheel measurement, in native encoder ticks per second. */
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
         * Returns whether this capture proves both wheels reached one positive accepted request.
         *
         * <p>This deliberately uses each member measurement rather than substituting the grouped
         * Plant's mean. It is controller evidence only: it does not prove flywheel balance under a
         * game piece, projectile release, or scoring.</p>
         */
        public boolean ready() {
            double requestedVelocity = requestedVelocityTicksPerSec();
            PlantTargetResolution resolution = plantSnapshot.targetResolution();
            return requestedVelocity > 0.0
                    && leftAtTarget()
                    && rightAtTarget()
                    && resolution.hasTarget()
                    && resolution.satisfiesIntent()
                    && !resolution.usedFallback()
                    && !resolution.clampedByPlanner()
                    && plantSnapshot.targetStatus().accepted()
                    && Double.compare(requestedVelocity,
                            plantSnapshot.requestedTarget()) == 0
                    && Double.compare(requestedVelocity,
                            plantSnapshot.appliedTarget()) == 0;
        }

        /** Returns the immutable grouped Plant capture for advanced diagnostics. */
        public PlantSnapshot plantSnapshot() {
            return plantSnapshot;
        }

        private boolean withinTolerance(double measuredVelocityTicksPerSec) {
            double requestedVelocity = requestedVelocityTicksPerSec();
            return Double.isFinite(measuredVelocityTicksPerSec)
                    && Double.isFinite(requestedVelocity)
                    && Math.abs(measuredVelocityTicksPerSec - requestedVelocity)
                    <= velocityToleranceTicksPerSec;
        }
    }

    /**
     * Replaces the persistent paired velocity request.
     *
     * @param velocityTicksPerSec finite request in the mechanism's configured inclusive range
     */
    void setVelocityTicksPerSec(double velocityTicksPerSec);

    /**
     * Builds a fresh single-use Task that requests a velocity and waits for both wheels.
     *
     * <p>Success requires a post-start publication for this request, command-correlated grouped
     * Plant arrival, and both independent measurements within tolerance. Timeout leaves the
     * persistent velocity request unchanged. Active cancellation requests {@code 0.0}; the later
     * normal output heartbeat realizes that request. Neither outcome proves an object launched.</p>
     *
     * @param velocityTicksPerSec finite request in the configured inclusive range
     * @param timeoutSec finite timeout strictly greater than zero, in seconds
     * @return a fresh Task; call this method again for another run
     */
    Task setVelocityTask(double velocityTicksPerSec, double timeoutSec);

    /** Returns the latest complete cached grouped and per-wheel publication. */
    Status status();
}
