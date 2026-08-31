package edu.ftcsushi.fw.localization;

/**
 * High-rate absolute pose estimator whose published trajectory exposes coordinate continuity.
 *
 * <p>This capability is narrower than {@link AbsolutePoseEstimator}. Sparse delayed measurement
 * estimators, such as an AprilTag-only pose estimator, remain ordinary absolute estimators rather
 * than claiming to publish a trajectory that can be interpolated. Motion predictors and final
 * corrected localizers implement this contract because they can publish one current trajectory
 * sample per loop.</p>
 *
 * <p>The segment identifier is local to one estimator instance and is opaque. Consumers may test
 * identifiers only for equality. A deliberate pose reset, coordinate rebase, or another known
 * discontinuity that can make interpolation untruthful must publish a different identifier. Normal
 * physical motion does not change it. A corrected estimator may keep ordinary accepted corrections
 * in one as-published segment even when it rebases a private predictor; the predictor and corrected
 * estimator describe distinct trajectories.</p>
 *
 * <p>{@link #getEstimate()} and {@link #trajectorySegmentId()} form one cached observation between
 * estimator mutations. Both methods must be side-effect-free and stable across repeated reads.
 * Implementations must change the identifier no later than publishing the first estimate in a new
 * segment. When a nontransactional reset may have affected downstream state before failing, the
 * implementation should conservatively start a new segment and fail closed rather than let a
 * historical consumer interpolate across uncertain coordinates.</p>
 */
public interface PoseTrajectoryEstimator extends AbsolutePoseEstimator {

    /**
     * Returns the opaque identifier of the trajectory segment containing the cached estimate.
     *
     * <p>The value has no ordering or global identity. It is meaningful only for equality checks
     * between observations from this exact estimator instance.</p>
     *
     * @return publisher-local trajectory segment identifier
     */
    long trajectorySegmentId();
}
