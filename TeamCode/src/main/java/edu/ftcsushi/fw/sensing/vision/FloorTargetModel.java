package edu.ftcsushi.fw.sensing.vision;

/**
 * Explicit horizontal-plane model for the reference point reported by a floor-object detector.
 * The supplied height is an assumption in inches above robot Z=0, not a measured ball center.
 * The camera must lie above this plane. No physical dimensions or safe range are guessed.
 */
public final class FloorTargetModel {
    private final double referenceHeightInches;
    private final double maxRangeInches;

    private FloorTargetModel(double height, double maxRange) {
        referenceHeightInches = height;
        maxRangeInches = maxRange;
    }

    /**
     * Uses an explicit finite non-negative reference height. Range is initially unlimited;
     * numerical near-horizon rejection still applies, and no physical range accuracy is promised.
     */
    public static FloorTargetModel atHeightInches(double referenceHeightInches) {
        if (!Double.isFinite(referenceHeightInches) || referenceHeightInches < 0) {
            throw new IllegalArgumentException("referenceHeightInches must be finite and >= 0");
        }
        return new FloorTargetModel(referenceHeightInches, Double.POSITIVE_INFINITY);
    }

    /** Adds an inclusive finite positive camera-to-reference-point line-of-sight range limit. */
    public FloorTargetModel withMaxRangeInches(double maxRangeInches) {
        if (!Double.isFinite(maxRangeInches) || maxRangeInches <= 0) {
            throw new IllegalArgumentException("maxRangeInches must be finite and > 0");
        }
        return new FloorTargetModel(referenceHeightInches, maxRangeInches);
    }

    /** Assumed reference height above the horizontal robot floor plane, in inches. */
    public double referenceHeightInches() { return referenceHeightInches; }
    /** Inclusive line-of-sight range bound, or positive infinity when no range bound was supplied. */
    public double maxRangeInches() { return maxRangeInches; }
}
