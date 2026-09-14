package edu.ftcsushi.fw.spatial;

import edu.ftcsushi.fw.core.geometry.Pose2d;

/**
 * Immutable geometry for a fixed tool's center window during one fixed-heading straight move.
 *
 * <p>Build through {@link #straightFrom(Pose2d)}, then query each estimated object center with
 * {@link #encounterFieldCenter(double, double)}. All positions and distances are in inches;
 * headings are counter-clockwise-positive radians. Robot/tool +X is forward and +Y is left.
 * The endpoint changes robot position, never its heading or the tool's robot-relative pose.</p>
 *
 * <p>The window contains acceptable <em>object-center positions</em>, not the physical intake
 * opening. Its author already accounts for ball size, uncertainty, and tested extra clearance.
 * There is no automatic radius inflation or shrinkage. A center at a physical intake edge may
 * bounce away; even a modeled encounter inside this chosen inner window is not capture evidence.
 * A fixed claw may use the same geometry, but grasp height, orientation, closing, and confirmation
 * are separate decisions.</p>
 *
 * <p>This constant-work analytic query does not sample a path, drive a robot, rank objects, infer
 * identity or freshness, or prove robot-body clearance or allowed-territory compliance. The caller
 * supplies stationary estimated centers in the segment's field frame and retains their evidence.
 * Turns, curves, and moving tools are outside this model. Repeated queries are independent and
 * side-effect-free; duplicate coordinates do not represent a unique-object count. Transforms and
 * clipping use ordinary double precision, without an added tolerance or an exact-arithmetic
 * guarantee at arbitrarily ill-conditioned boundaries.</p>
 */
public final class ToolSweep2d {
    private final Pose2d fieldToToolStart;
    private final double fieldEndXInches;
    private final double fieldEndYInches;
    private final double toolCos;
    private final double toolSin;
    private final double toolDeltaForwardInches;
    private final double toolDeltaLeftInches;
    private final double travelInches;
    private final double minForwardInches;
    private final double maxForwardInches;
    private final double halfWidthInches;

    private ToolSweep2d(WindowAnswer answer, double minForwardInches,
                        double maxForwardInches, double halfWidthInches) {
        this.fieldToToolStart = answer.fieldToToolStart;
        this.fieldEndXInches = answer.fieldEndXInches;
        this.fieldEndYInches = answer.fieldEndYInches;
        this.toolCos = answer.toolCos;
        this.toolSin = answer.toolSin;
        this.toolDeltaForwardInches = answer.toolDeltaForwardInches;
        this.toolDeltaLeftInches = answer.toolDeltaLeftInches;
        this.travelInches = answer.travelInches;
        this.minForwardInches = minForwardInches;
        this.maxForwardInches = maxForwardInches;
        this.halfWidthInches = halfWidthInches;
    }

    /**
     * Starts the sole construction path with robot field position and its one fixed heading.
     *
     * @param fieldToRobotStart finite immutable field-to-robot pose, in inches and radians
     * @return an independent immutable stage requiring the robot-center endpoint
     * @throws NullPointerException if {@code fieldToRobotStart} is null
     * @throws IllegalArgumentException if any pose component is non-finite
     */
    public static EndpointStage straightFrom(Pose2d fieldToRobotStart) {
        return new EndpointAnswer(SpatialValidation.requireFinitePose2d(
                "fieldToRobotStart", fieldToRobotStart));
    }

    /** Required robot-center endpoint; retaining or reusing a stage never mutates another answer. */
    public interface EndpointStage {
        /**
         * Describes the end field position while retaining the starting robot heading.
         * Forward, reverse, sideways, diagonal, and stationary segments are all supported geometry.
         *
         * @param fieldXInches finite endpoint field X in inches
         * @param fieldYInches finite endpoint field Y in inches
         * @return independent stage requiring the fixed tool transform
         * @throws IllegalArgumentException if coordinates, displacement, or travel are non-finite
         */
        ToolStage toFieldPoint(double fieldXInches, double fieldYInches);
    }

    /** Required fixed tool pose; an intake and a fixed claw use the same geometry vocabulary. */
    public interface ToolStage {
        /**
         * Applies a rigid robot-to-tool transform exactly once.
         *
         * <p>Composition follows {@link Pose2d#then(Pose2d)} without normalizing headings.
         * The tool cannot move relative to the robot during this modeled segment.</p>
         *
         * @param robotToTool finite robot-to-tool pose, in inches and radians
         * @return independent stage requiring acceptable object-center bounds
         * @throws NullPointerException if {@code robotToTool} is null
         * @throws IllegalArgumentException if the pose or required derived transform is non-finite
         */
        CenterWindowStage throughTool(Pose2d robotToTool);
    }

    /** Required already-inset object-center window, expressed in the fixed tool's local frame. */
    public interface CenterWindowStage {
        /**
         * Finishes the sweep with closed acceptable center bounds in tool coordinates.
         *
         * <p>The forward interval is [{@code minForwardInches}, {@code maxForwardInches}];
         * the left interval is [{@code -fullWidthInches/2}, {@code +fullWidthInches/2}]. Equal
         * forward bounds permit a zero-depth line. Negative bounds deliberately place part or all
         * of the window behind the tool origin. Boundaries are included without extra tolerance.
         * These are center bounds, not physical opening dimensions; no ball-size adjustment is
         * performed. Geometric sideways/reverse encounters need not collect anything physically.</p>
         *
         * @param minForwardInches finite minimum acceptable center-forward coordinate in inches
         * @param maxForwardInches finite maximum center-forward coordinate, at least the minimum
         * @param fullWidthInches finite positive full center-window width, with nonzero half-width
         * @return independent immutable sweep
         * @throws IllegalArgumentException if bounds are non-finite or reversed, or width is not
         *         positive with a representable positive half-width
         */
        ToolSweep2d centerWindowInches(double minForwardInches, double maxForwardInches,
                                       double fullWidthInches);
    }

    /**
     * Returns the continuous interval during which one estimated field center is in the window.
     *
     * <p>Entry and exit are robot-center travel distances along the authored segment, not time,
     * observed progress, tool-to-target range, capture, or clearance. A stationary in-window
     * center has entry and exit zero. A tangency is included and may also have equal distances.
     * A valid miss is distinct from invalid or nonrepresentable input geometry.</p>
     *
     * @param fieldXInches estimated center's finite field X in inches
     * @param fieldYInches estimated center's finite field Y in inches
     * @return immutable encounter or miss; distance access on a miss throws
     * @throws IllegalArgumentException if coordinates or required derived math are non-finite,
     *         or a positive encounter distance cannot be represented
     */
    public Encounter encounterFieldCenter(double fieldXInches, double fieldYInches) {
        SpatialValidation.requireFinite("center fieldXInches", fieldXInches);
        SpatialValidation.requireFinite("center fieldYInches", fieldYInches);
        double fieldOffsetX = finiteDerived("center field X offset", fieldXInches - fieldToToolStart.xInches);
        double fieldOffsetY = finiteDerived("center field Y offset", fieldYInches - fieldToToolStart.yInches);
        double forward = finiteDerived("center tool-forward coordinate",
                toolCos * fieldOffsetX + toolSin * fieldOffsetY);
        double left = finiteDerived("center tool-left coordinate",
                -toolSin * fieldOffsetX + toolCos * fieldOffsetY);
        // Validate both end coordinates before clipping: invalid geometry is never an ordinary miss.
        double endForward = finiteDerived("end center tool-forward coordinate", forward - toolDeltaForwardInches);
        double endLeft = finiteDerived("end center tool-left coordinate", left - toolDeltaLeftInches);
        Interval interval = new Interval(travelInches);
        if (!interval.clip(forward, endForward, toolDeltaForwardInches, minForwardInches, maxForwardInches)
                || !interval.clip(left, endLeft, toolDeltaLeftInches, -halfWidthInches, halfWidthInches)) {
            return Encounter.MISS;
        }
        return new Encounter(true, interval.entry, interval.exit);
    }

    /** Immutable modeled encounter interval; has no object identity, evidence, or lifecycle. */
    public static final class Encounter {
        private static final Encounter MISS = new Encounter(false, 0.0, 0.0);
        private final boolean hasEncounter;
        private final double entryTravelInches;
        private final double exitTravelInches;

        private Encounter(boolean hasEncounter, double entryTravelInches, double exitTravelInches) {
            this.hasEncounter = hasEncounter;
            this.entryTravelInches = entryTravelInches;
            this.exitTravelInches = exitTravelInches;
        }

        /** Returns whether the supplied center enters or touches the authored inner window. */
        public boolean hasEncounter() { return hasEncounter; }

        /**
         * Returns finite first robot-center travel in inches, including the start/end boundaries.
         * @throws IllegalStateException if {@link #hasEncounter()} is false
         */
        public double entryTravelInches() {
            requireEncounter();
            return entryTravelInches;
        }

        /**
         * Returns finite last robot-center travel in inches, no earlier than entry.
         * @throws IllegalStateException if {@link #hasEncounter()} is false
         */
        public double exitTravelInches() {
            requireEncounter();
            return exitTravelInches;
        }

        /** Rejects fabricated distances for an ordinary geometric miss. */
        private void requireEncounter() {
            if (!hasEncounter) {
                throw new IllegalStateException("No center-window encounter; check hasEncounter() before travel distances");
            }
        }

        /** Returns geometry only, never a capture or clearance status. */
        @Override public String toString() {
            return hasEncounter ? "Encounter{entryTravelInches=" + entryTravelInches
                    + ", exitTravelInches=" + exitTravelInches + '}' : "Encounter{miss}";
        }
    }

    /** Returns the fixed tool geometry and modeled robot-center travel, without sampling anything. */
    @Override public String toString() {
        return "ToolSweep2d{fieldToToolStart=" + fieldToToolStart + ", robotEndFieldXInches="
                + fieldEndXInches + ", robotEndFieldYInches=" + fieldEndYInches
                + ", travelInches=" + travelInches + ", centerForwardInches=[" + minForwardInches
                + ", " + maxForwardInches + "], centerHalfWidthInches=" + halfWidthInches + '}';
    }

    /** Immutable first-stage answer. */
    private static final class EndpointAnswer implements EndpointStage {
        private final Pose2d fieldToRobotStart;
        private EndpointAnswer(Pose2d fieldToRobotStart) { this.fieldToRobotStart = fieldToRobotStart; }

        @Override public ToolStage toFieldPoint(double fieldXInches, double fieldYInches) {
            SpatialValidation.requireFinite("endpoint fieldXInches", fieldXInches);
            SpatialValidation.requireFinite("endpoint fieldYInches", fieldYInches);
            double deltaX = finiteDerived("robot-center field X displacement", fieldXInches - fieldToRobotStart.xInches);
            double deltaY = finiteDerived("robot-center field Y displacement", fieldYInches - fieldToRobotStart.yInches);
            double travel = finiteDerived("robot-center travelInches", Math.hypot(deltaX, deltaY));
            return new ToolAnswer(fieldToRobotStart, fieldXInches, fieldYInches, deltaX, deltaY, travel);
        }
    }

    /** Immutable segment answer; no tool or window is retained until supplied. */
    private static final class ToolAnswer implements ToolStage {
        private final Pose2d fieldToRobotStart;
        private final double fieldEndXInches;
        private final double fieldEndYInches;
        private final double deltaXInches;
        private final double deltaYInches;
        private final double travelInches;
        private ToolAnswer(Pose2d start, double endX, double endY, double deltaX, double deltaY, double travel) {
            fieldToRobotStart = start;
            fieldEndXInches = endX;
            fieldEndYInches = endY;
            deltaXInches = deltaX;
            deltaYInches = deltaY;
            travelInches = travel;
        }

        @Override public CenterWindowStage throughTool(Pose2d robotToTool) {
            SpatialValidation.requireFinitePose2d("robotToTool", robotToTool);
            Pose2d start = fieldToRobotStart.then(robotToTool);
            SpatialValidation.requireFinitePose2d("derived fieldToToolStart", start);
            SpatialValidation.requireFinitePose2d("derived fieldToToolEnd",
                    fieldToRobotStart.withTranslation(fieldEndXInches, fieldEndYInches).then(robotToTool));
            // A rigid offset is applied only once; the translation vector gets rotation only.
            double cos = Math.cos(start.headingRad);
            double sin = Math.sin(start.headingRad);
            double forward = finiteDerived("tool-forward displacement", cos * deltaXInches + sin * deltaYInches);
            double left = finiteDerived("tool-left displacement", -sin * deltaXInches + cos * deltaYInches);
            return new WindowAnswer(start, fieldEndXInches, fieldEndYInches, cos, sin, forward, left, travelInches);
        }
    }

    /** Immutable transform answer reused safely for independently sized windows. */
    private static final class WindowAnswer implements CenterWindowStage {
        private final Pose2d fieldToToolStart;
        private final double fieldEndXInches;
        private final double fieldEndYInches;
        private final double toolCos;
        private final double toolSin;
        private final double toolDeltaForwardInches;
        private final double toolDeltaLeftInches;
        private final double travelInches;
        private WindowAnswer(Pose2d start, double endX, double endY, double cos, double sin,
                              double forward, double left, double travel) {
            fieldToToolStart = start;
            fieldEndXInches = endX;
            fieldEndYInches = endY;
            toolCos = cos;
            toolSin = sin;
            toolDeltaForwardInches = forward;
            toolDeltaLeftInches = left;
            travelInches = travel;
        }

        @Override public ToolSweep2d centerWindowInches(double minForwardInches, double maxForwardInches,
                                                        double fullWidthInches) {
            SpatialValidation.requireFinite("minForwardInches", minForwardInches);
            SpatialValidation.requireFinite("maxForwardInches", maxForwardInches);
            SpatialValidation.requireFinite("fullWidthInches", fullWidthInches);
            if (minForwardInches > maxForwardInches) {
                throw new IllegalArgumentException("centerWindowInches requires minForwardInches <= maxForwardInches");
            }
            if (fullWidthInches <= 0.0 || fullWidthInches / 2.0 == 0.0) {
                throw new IllegalArgumentException("fullWidthInches must be > 0 with a representable positive half-width");
            }
            return new ToolSweep2d(this, minForwardInches, maxForwardInches, fullWidthInches / 2.0);
        }
    }

    /** Local per-query analytic interval, expressed in travel to avoid underflowing tiny fractions. */
    private static final class Interval {
        private final double travel;
        private double entry;
        private double exit;
        private Interval(double travel) { this.travel = travel; this.exit = travel; }

        /** Clips one linear coordinate against closed bounds without dividing an irrelevant crossing. */
        private boolean clip(double start, double end, double delta, double min, double max) {
            if (Math.max(start, end) < min || Math.min(start, end) > max) return false;
            if (delta == 0.0) return start >= min && start <= max;
            if (delta > 0.0) {
                if (start > max) entry = Math.max(entry, crossingTravel(start - max, delta, travel));
                if (end < min) exit = Math.min(exit, crossingTravel(start - min, delta, travel));
            } else {
                if (start < min) entry = Math.max(entry, crossingTravel(min - start, -delta, travel));
                if (end > max) exit = Math.min(exit, crossingTravel(max - start, -delta, travel));
            }
            return entry <= exit;
        }
    }

    /** Computes a crossing known to lie on the segment without overflowing or losing a tiny fraction. */
    private static double crossingTravel(double numerator, double axisTravel, double travel) {
        finiteDerived("center-window crossing displacement", numerator);
        if (numerator == 0.0) return 0.0;
        double fraction = numerator / axisTravel;
        double result;
        if (fraction >= Double.MIN_NORMAL) {
            result = fraction * travel;
        } else {
            // numerator * travel / axisTravel, scaling all three to retain subnormal fractions.
            int ne = Math.getExponent(numerator);
            int te = Math.getExponent(travel);
            int ae = Math.getExponent(axisTravel);
            double scaled = Math.scalb(numerator, -ne) * Math.scalb(travel, -te)
                    / Math.scalb(axisTravel, -ae);
            result = Math.scalb(scaled, ne + te - ae);
        }
        finiteDerived("center-window crossing travelInches", result);
        if (result == 0.0) {
            throw new IllegalArgumentException("Positive center-window crossing travel is too small to represent; rescale geometry");
        }
        // Endpoint comparisons proved this crossing lies on the closed finite segment.
        return Math.min(result, travel);
    }

    /** Rejects unrepresentable derived values rather than silently treating them as misses. */
    private static double finiteDerived(String name, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " is not finitely representable; use a smaller consistent coordinate scale");
        }
        return value;
    }
}
