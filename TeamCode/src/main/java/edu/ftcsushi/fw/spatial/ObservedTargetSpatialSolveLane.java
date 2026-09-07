package edu.ftcsushi.fw.spatial;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;

/**
 * Delayed robot-at-capture visual feedback for {@link References#observedPoint} references.
 *
 * <p>No field pose is invented and no motion compensation is promised. The selected observation's
 * freshness policy remains authoritative. Rigid tools work directly; a moving tool requires a
 * truthful capture-time frame provider. The lane borrows sources and has no lifecycle to reset.</p>
 */
public final class ObservedTargetSpatialSolveLane implements SpatialSolveLane {
    /** Creates the stateless observation-backed lane. */
    public ObservedTargetSpatialSolveLane() { }

    @Override
    public SpatialLaneResult solve(SpatialSolveRequest request) {
        TargetObservation2d translationTarget = observation(request.translationTarget, request);
        TargetObservation2d facingTarget = observation(request.facingTarget, request);
        TranslationSolution translation = null;
        FacingSolution facing = null;
        if (translationTarget != null && translationTarget.hasPosition()) {
            Pose2d frame = request.robotToTranslationFrameAt(translationTarget.timestamp);
            double range = Math.hypot(translationTarget.forwardInches, translationTarget.leftInches);
            if (SpatialValidation.isFinite(frame) && Double.isFinite(range)) {
                translation = SpatialSolveMath.translationFromRobotPoint(frame,
                    new Pose2d(translationTarget.forwardInches, translationTarget.leftInches, 0.0),
                    true, range,
                    translationTarget.quality, translationTarget.timestamp)
                    .withDirectObservationEvidence(translationTarget.timestamp);
                if (!SpatialValidation.isFinite(translation.translationFrameToTargetPoint)
                        || !Double.isFinite(translation.frameDistanceInches())) translation = null;
            }
        }
        if (facingTarget != null && facingTarget.hasPosition()) {
            Pose2d frame = request.robotToFacingFrameAt(facingTarget.timestamp);
            Pose2d point = new Pose2d(facingTarget.forwardInches, facingTarget.leftInches, 0.0);
            if (SpatialValidation.isFinite(frame) && SpatialValidation.isFinite(frame.inverse().then(point))) {
                facing = SpatialSolveMath.facingFromRobotPoint(frame,
                    new Pose2d(facingTarget.forwardInches, facingTarget.leftInches, 0.0),
                    facingTarget.quality, facingTarget.timestamp)
                    .withDirectObservationEvidence(facingTarget.timestamp);
                if (!Double.isFinite(facing.facingErrorRad)) facing = null;
            }
        }
        return SpatialLaneResult.of(translation, facing, null, null);
    }

    private static TargetObservation2d observation(Object target, SpatialSolveRequest request) {
        if (!(target instanceof SpatialTargets.ReferencePointTarget)) return null;
        ReferencePoint2d reference = ((SpatialTargets.ReferencePointTarget) target).reference;
        if (!(reference instanceof References.ObservedPointRef)) return null;
        TargetSelectionResult selected = ((References.ObservedPointRef) reference).get(request.clock);
        return selected.isUsable(request.clock) ? selected.observation() : null;
    }
}
