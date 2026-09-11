package edu.ftcsushi.robots.examples.visionmemory;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.FieldTargetMemory;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionSource;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.spatial.ReferencePoint2d;
import edu.ftcsushi.fw.spatial.References;
import edu.ftcsushi.fw.spatial.SpatialQuery;
import edu.ftcsushi.fw.spatial.SpatialQueryResult;
import edu.ftcsushi.fw.spatial.SpatialSolveSet;
import edu.ftcsushi.fw.spatial.SpatialTargets;

/**
 * Independent hardware-neutral example: remember recent locations and inspect one spatial answer.
 *
 * <p>This read-only service owns its memory, selector, and query. It borrows already projected
 * frames and the same authoritative trajectory estimator used for that projection. Register the
 * camera/localization/history owners before this service. Their normal update and history record
 * precede this service; presenters read only {@link #status()} afterward. No drive, intake, Task,
 * physical camera ownership, or second localization update is introduced.</p>
 *
 * <p>The constructor's explicit numbers are a synthetic software fixture, not physical defaults.
 * Adapting the example requires reviewed robot-specific retention, matching, capacity, and pose
 * evidence limits. A remembered location is not a verified ball or fresh pickup permission.</p>
 */
public final class RecentFieldLocations implements RobotProgram.Service {

    /** Immutable publication; earlier publications remain historical, not live control decisions. */
    public static final class Status {
        /** Exact last memory publication, including capture and input-decision evidence. */
        public final FieldTargetMemory.Snapshot memory;
        /**
         * Exact query publication, or null before an update and after reset/STOP. A non-null query
         * can still have no solved channels; inspect its lane rather than assuming arrival.
         */
        public final SpatialQueryResult geometry;

        private Status(FieldTargetMemory.Snapshot memory, SpatialQueryResult geometry) {
            this.memory = memory;
            this.geometry = geometry;
        }
    }

    private final PoseTrajectoryEstimator localization;
    private final FieldTargetMemory memory;
    private final FieldTargetSelectionSource selected;
    private final SpatialQuery query;
    private Status status;
    private boolean segmentKnown;
    private long segment;
    private boolean stopped;

    /**
     * Constructs the complete read-only graph without sampling either borrowed input.
     *
     * @param fieldObjects one fixed producer's anonymous located objects projected through
     *                     capture-time history, not a field map or mixed-camera collection
     * @param localization authoritative trajectory already advanced by its own service
     */
    public RecentFieldLocations(Source<TargetObservations2d> fieldObjects,
                                PoseTrajectoryEstimator localization) {
        this.localization = Objects.requireNonNull(localization, "localization");
        memory = FieldTargetMemory.fromFieldObjects(fieldObjects)
                .retainingForSec(1.0)
                .matchingWithinInches(2.0)
                .maxEntries(4);
        selected = TargetSelections.fromRecentFieldLocations(memory.source())
                .choose(FieldTargetSelectionPolicies.nearestToRobot(localization, 0.20, 0.10));
        ReferencePoint2d point = References.selectedFieldTargetPoint(selected);
        query = SpatialQuery.builder()
                .translateTo(SpatialTargets.point(point))
                .andFaceTo(SpatialTargets.point(point))
                .solveWith(SpatialSolveSet.builder()
                        .absolutePose(localization, 0.20, 0.10).build())
                .build();
        status = new Status(memory.snapshot(), null);
    }

    /** Returns only cached immutable evidence; this never polls a camera or advances a query. */
    public Status status() {
        return status;
    }

    /**
     * Checks known coordinate discontinuity, advances memory once, then publishes read-only geometry.
     * Repeated calls share the memory and query's same-cycle guards. Exceptions propagate to the
     * managed host; they do not turn the previous status into accepted current evidence.
     */
    @Override
    public void update(LoopClock clock) {
        if (stopped) return;
        Objects.requireNonNull(clock, "clock");
        long currentSegment = localization.trajectorySegmentId();
        if (stopped) return;
        if (segmentKnown && segment != currentSegment) resetBeforeTransition(clock);
        segment = currentSegment;
        segmentKnown = true;
        memory.update(clock);
        if (stopped) return;
        SpatialQueryResult geometry = query.get(clock);
        if (!stopped) status = new Status(memory.snapshot(), geometry);
    }

    /**
     * Invalidates locally owned evidence before an external coordinate or camera transition.
     *
     * <p>Call before rebasing pose, resetting history, or changing this producer's camera/pipeline
     * configuration, even if that operation might throw. Memory fences captures at/before this
     * boundary. The automatic segment check additionally catches known trajectory changes before
     * sampling, but cannot discover out-of-band camera changes. This method neither performs the
     * external transition nor resets any borrowed source. Complete it before downstream consumers
     * run; old query publications are not retroactively rewritten. A new producer/field graph
     * requires a new service.</p>
     *
     * @param clock the same program clock used by updates
     */
    public void resetBeforeTransition(LoopClock clock) {
        if (stopped) return;
        memory.reset(clock);
        selected.reset();
        query.reset();
        status = new Status(memory.snapshot(), null);
    }

    /** Permanently revokes owned memory; safe before START and repeatedly; borrowed owners survive. */
    @Override
    public void stop() {
        if (stopped) return;
        stopped = true;
        memory.stop();
        status = new Status(memory.snapshot(), null);
    }
}
