package edu.ftcsushi.fw.sensing.vision;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.geometry.Vec3;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;

/**
 * Pure calibrated camera-ray intersection with an explicitly assumed horizontal target plane.
 * Angles are separate image-plane angles, not spherical yaw/elevation. Output is robot-relative
 * at capture, in inches, with no invented identity, heading, confidence, or physical-center claim.
 */
public final class FloorTargetProjection {
    /** Rejects numerically ill-conditioned near-horizon rays after unit normalization. */
    private static final double MIN_DOWN_COMPONENT = 1.0e-9;

    /** Exact software reason a reference point could or could not be projected. */
    public enum Reason {
        AVAILABLE, INVALID_ANGLES, INVALID_RAY, TIMESTAMP_UNAVAILABLE,
        CAMERA_NOT_ABOVE_PLANE, PARALLEL_OR_UPWARD_RAY, NO_FORWARD_INTERSECTION,
        RANGE_EXCEEDED, NONFINITE_POSITION
    }

    /** Immutable projection result. An unavailable result contains no target observation. */
    public static final class Result {
        private final TargetObservation2d observation;
        private final Reason reason;

        private Result(TargetObservation2d observation, Reason reason) {
            this.observation = observation;
            this.reason = reason;
        }
        /** Whether the declared geometric model produced finite position. */
        public boolean isAvailable() { return reason == Reason.AVAILABLE; }
        /** Located observation, or no-target when projection was rejected. */
        public TargetObservation2d observation() { return observation; }
        /** Exact geometric rejection reason, or AVAILABLE. */
        public Reason reason() { return reason; }
    }

    private FloorTargetProjection() { }

    /**
     * Projects separate horizontal-left/vertical-up plane angles in radians. Each angle must lie
     * strictly between -pi/2 and pi/2. The ray is proportional to (1, tan(horizontal), tan(vertical)).
     * Runtime invalid angles produce unavailable; required collaborators must be non-null.
     */
    public static Result projectAngles(double horizontalLeftRad, double verticalUpRad,
                                       CameraMountConfig mount, FloorTargetModel model,
                                       LoopTimestamp timestamp) {
        requireInputs(mount, model, timestamp);
        if (!Double.isFinite(horizontalLeftRad) || !Double.isFinite(verticalUpRad)
                || Math.abs(horizontalLeftRad) >= Math.PI / 2
                || Math.abs(verticalUpRad) >= Math.PI / 2) return unavailable(Reason.INVALID_ANGLES);
        return projectRay(new Vec3(1.0, Math.tan(horizontalLeftRad), Math.tan(verticalUpRad)),
                mount, model, timestamp);
    }

    /**
     * Projects a nonzero calibrated camera ray (+X forward, +Y left, +Z up). Positive scaling
     * does not affect the result. A lens-forward ray and camera above the target plane are required.
     * The original timestamp is retained; consumers still gate its age/reset validity with their
     * shared clock. No localization or hardware is sampled here.
     */
    public static Result projectRay(Vec3 cameraRay, CameraMountConfig mount,
                                    FloorTargetModel model, LoopTimestamp timestamp) {
        requireInputs(mount, model, timestamp);
        Objects.requireNonNull(cameraRay, "cameraRay");
        if (!timestamp.isAvailable()) return unavailable(Reason.TIMESTAMP_UNAVAILABLE);
        if (!finite(cameraRay) || cameraRay.x <= 0) return unavailable(Reason.INVALID_RAY);
        Vec3 unitCamera = normalized(cameraRay);
        if (unitCamera == null) return unavailable(Reason.INVALID_RAY);
        Pose3d camera = mount.robotToCameraPose();
        if (camera.zInches <= model.referenceHeightInches()) {
            return unavailable(Reason.CAMERA_NOT_ABOVE_PLANE);
        }
        Vec3 ray = normalized(camera.rotation().mul(unitCamera));
        if (ray == null) return unavailable(Reason.INVALID_RAY);
        if (ray.z >= -MIN_DOWN_COMPONENT) return unavailable(Reason.PARALLEL_OR_UPWARD_RAY);
        double range = (model.referenceHeightInches() - camera.zInches) / ray.z;
        if (!Double.isFinite(range)) return unavailable(Reason.NONFINITE_POSITION);
        if (range <= 0) return unavailable(Reason.NO_FORWARD_INTERSECTION);
        if (range > model.maxRangeInches()) return unavailable(Reason.RANGE_EXCEEDED);
        double forward = camera.xInches + range * ray.x;
        double left = camera.yInches + range * ray.y;
        if (!Double.isFinite(forward) || !Double.isFinite(left)) {
            return unavailable(Reason.NONFINITE_POSITION);
        }
        return new Result(TargetObservation2d.ofRobotRelativePosition(
                forward, left, Double.NaN, timestamp), Reason.AVAILABLE);
    }

    private static void requireInputs(CameraMountConfig mount, FloorTargetModel model,
                                      LoopTimestamp timestamp) {
        Objects.requireNonNull(mount, "mount");
        Objects.requireNonNull(model, "model");
        Objects.requireNonNull(timestamp, "timestamp");
    }

    private static Result unavailable(Reason reason) {
        return new Result(TargetObservation2d.none(), reason);
    }

    private static boolean finite(Vec3 vector) {
        return Double.isFinite(vector.x) && Double.isFinite(vector.y) && Double.isFinite(vector.z);
    }

    private static Vec3 normalized(Vec3 vector) {
        if (!finite(vector)) return null;
        double scale = Math.max(Math.abs(vector.x), Math.max(Math.abs(vector.y), Math.abs(vector.z)));
        if (scale == 0) return null;
        double x = vector.x / scale;
        double y = vector.y / scale;
        double z = vector.z / scale;
        double length = Math.sqrt(x * x + y * y + z * z);
        return new Vec3(x / length, y / length, z / length);
    }
}
