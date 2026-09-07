package edu.ftcsushi.robots.examples.pedro.adaptive;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.FloorTargetModel;
import edu.ftcsushi.fw.sensing.vision.FloorTargetProjection;

/** Robot-owned band ranking over the shared core's capture-time floor projection. */
final class AdaptiveCollectionProjection {
    /** This example explicitly intersects the floor, not a physical object's center. */
    private static final FloorTargetModel FLOOR = FloorTargetModel.atHeightInches(0.0);

    static final class Result {
        final int projectablePointCount;
        final int inBoxPointCount;
        final double bandStartYInches;
        final double bandEndYInches;
        final int bandPointCount;

        Result(int projectablePointCount, int inBoxPointCount,
               double bandStartYInches, double bandEndYInches, int bandPointCount) {
            this.projectablePointCount = projectablePointCount;
            this.inBoxPointCount = inBoxPointCount;
            this.bandStartYInches = bandStartYInches;
            this.bandEndYInches = bandEndYInches;
            this.bandPointCount = bandPointCount;
        }
    }

    private AdaptiveCollectionProjection() {
    }

    static Result select(List<AdaptiveCollectionVisionService.DetectorAngles> detections,
                         PlanarPoseHistory.Lookup fieldPoseLookup, CameraMountConfig cameraMount,
                         double minX, double maxX, double minY, double maxY, double bandWidth) {
        ArrayList<Double> inBoxY = new ArrayList<Double>();
        int projectable = 0;

        for (AdaptiveCollectionVisionService.DetectorAngles detection : detections) {
            FloorTargetProjection.Result projection = FloorTargetProjection.projectAngles(
                    -Math.toRadians(detection.horizontalRightDeg),
                    Math.toRadians(detection.verticalUpDeg),
                    cameraMount, FLOOR, fieldPoseLookup.timestamp());
            if (!projection.isAvailable()) continue;
            TargetObservation2d point = projection.observation().withFieldPoseLookup(fieldPoseLookup);
            if (!point.hasFieldPosition()) continue;
            projectable++;
            if (point.fieldXInches >= minX && point.fieldXInches <= maxX
                    && point.fieldYInches >= minY && point.fieldYInches <= maxY) {
                inBoxY.add(point.fieldYInches);
            }
        }
        if (inBoxY.isEmpty()) {
            return new Result(projectable, 0, Double.NaN, Double.NaN, 0);
        }

        SortedSet<Double> starts = new TreeSet<Double>();
        double maxStart = maxY - bandWidth;
        for (double y : inBoxY) {
            starts.add(clamp(y, minY, maxStart));
            starts.add(clamp(y - bandWidth, minY, maxStart));
        }
        double bestStart = starts.first();
        int bestCount = -1;
        for (double start : starts) {
            int count = count(inBoxY, start, start + bandWidth);
            if (count > bestCount || (count == bestCount && start < bestStart)) {
                bestStart = start;
                bestCount = count;
            }
        }
        return new Result(projectable, inBoxY.size(), bestStart,
                bestStart + bandWidth, bestCount);
    }

    private static int count(List<Double> values, double start, double end) {
        int count = 0;
        for (double value : values) if (value >= start && value <= end) count++;
        return count;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
