package edu.ftcphoenix.fw.ftc;

import edu.ftcphoenix.fw.spatial.AxisAlignedBoxRegion2d;
import edu.ftcphoenix.fw.spatial.ConvexRegion2d;

/**
 * FTC field-region helpers shared by localization, guidance, and testers.
 *
 * <p>Most FTC games use the standard 12 ft x 12 ft square field with the field-frame origin at
 * the center. That means the full floor footprint spans {@code [-72, +72]} inches on both X and Y.
 * These helpers keep that geometry in one framework-owned place instead of duplicating the same
 * literal bounds across robot code and test tools.</p>
 */
public final class FtcFieldRegions {

    /**
     * Half-width of the standard FTC field, in inches.
     */
    public static final double HALF_FIELD_INCHES = 72.0;

    private FtcFieldRegions() {
        // utility holder
    }

    /**
     * Returns the standard full FTC field floor footprint.
     */
    public static ConvexRegion2d fullField() {
        return fullFieldWithMargin(0.0);
    }

    /**
     * Returns the standard FTC field floor footprint expanded (positive margin) or shrunk
     * (negative margin) by {@code marginInches} on each side.
     *
     * <p>This is useful when a caller wants a small tolerance band for plausibility gating rather
     * than a perfectly hard wall at the official boundary.</p>
     */
    public static ConvexRegion2d fullFieldWithMargin(double marginInches) {
        if (!Double.isFinite(marginInches)) {
            throw new IllegalArgumentException("marginInches must be finite");
        }
        double h = HALF_FIELD_INCHES + marginInches;
        if (h <= 0.0) {
            throw new IllegalArgumentException(
                    "marginInches shrinks the FTC field to zero/negative size: " + marginInches
            );
        }
        return new AxisAlignedBoxRegion2d(-h, h, -h, h);
    }
}

