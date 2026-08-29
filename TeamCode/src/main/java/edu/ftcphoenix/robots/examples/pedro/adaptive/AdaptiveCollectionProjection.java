package edu.ftcphoenix.robots.examples.pedro.adaptive;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import edu.ftcphoenix.fw.core.geometry.Mat3;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.geometry.Vec3;

/** Pure package-private floor projection and deterministic fixed-width band ranking. */
final class AdaptiveCollectionProjection {
    private static final double PARALLEL_RAY_EPSILON = 1.0e-9;

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
                         Pose3d fieldToRobotPose, Pose3d robotToCameraPose,
                         double minX, double maxX, double minY, double maxY, double bandWidth) {
        Pose3d fieldToCamera = fieldToRobotPose.then(robotToCameraPose);
        Vec3 origin = fieldToCamera.translation();
        if (!finite(origin)) return new Result(0, 0, Double.NaN, Double.NaN, 0);
        Mat3 cameraToFieldRotation = fieldToCamera.rotation();
        ArrayList<Double> inBoxY = new ArrayList<Double>();
        int projectable = 0;

        for (AdaptiveCollectionVisionService.DetectorAngles detection : detections) {
            Vec3 cameraRay = cameraRay(detection);
            if (cameraRay == null) continue;
            Vec3 fieldRay = cameraToFieldRotation.mul(cameraRay);
            if (!finite(fieldRay) || fieldRay.z >= -PARALLEL_RAY_EPSILON) continue;

            double distance = -origin.z / fieldRay.z;
            if (!Double.isFinite(distance) || distance <= 0.0) continue;
            Vec3 point = origin.add(fieldRay.scale(distance));
            if (!finite(point)) continue;
            projectable++;
            if (point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY) {
                inBoxY.add(point.y);
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

    private static Vec3 cameraRay(AdaptiveCollectionVisionService.DetectorAngles detection) {
        if (!Double.isFinite(detection.horizontalRightDeg)
                || !Double.isFinite(detection.verticalDownDeg)) return null;
        double left = -Math.tan(Math.toRadians(detection.horizontalRightDeg));
        double up = -Math.tan(Math.toRadians(detection.verticalDownDeg));
        return Double.isFinite(left) && Double.isFinite(up) ? new Vec3(1.0, left, up) : null;
    }

    private static int count(List<Double> values, double start, double end) {
        int count = 0;
        for (double value : values) if (value >= start && value <= end) count++;
        return count;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static boolean finite(Vec3 vector) {
        return Double.isFinite(vector.x) && Double.isFinite(vector.y) && Double.isFinite(vector.z);
    }
}
