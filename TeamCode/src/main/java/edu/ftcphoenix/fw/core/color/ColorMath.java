package edu.ftcphoenix.fw.core.color;

/**
 * Shared RGB-derived math used by Phoenix color value objects.
 *
 * <p>This package-private helper keeps {@link Rgba} and {@link NormalizedRgba} parallel without
 * duplicating the same max/min/chroma/HSV math in two places.</p>
 */
final class ColorMath {

    private ColorMath() {
        // utility class
    }

    static double maxChannel(double r, double g, double b) {
        return Math.max(r, Math.max(g, b));
    }

    static double minChannel(double r, double g, double b) {
        return Math.min(r, Math.min(g, b));
    }

    static double chroma(double r, double g, double b) {
        return maxChannel(r, g, b) - minChannel(r, g, b);
    }

    static double ratio(double channel, double sumRgb) {
        return (sumRgb <= 0.0) ? 0.0 : (channel / sumRgb);
    }

    static double hueDeg(double r, double g, double b) {
        double max = maxChannel(r, g, b);
        double chroma = chroma(r, g, b);
        if (chroma <= 0.0) {
            return 0.0;
        }

        double hue;
        if (max == r) {
            hue = 60.0 * ((g - b) / chroma);
        } else if (max == g) {
            hue = 60.0 * (((b - r) / chroma) + 2.0);
        } else {
            hue = 60.0 * (((r - g) / chroma) + 4.0);
        }

        if (hue < 0.0) {
            hue += 360.0;
        }
        return hue;
    }

    static double saturation(double r, double g, double b) {
        double value = value(r, g, b);
        if (value <= 0.0) {
            return 0.0;
        }
        return chroma(r, g, b) / value;
    }

    static double value(double r, double g, double b) {
        return maxChannel(r, g, b);
    }
}
