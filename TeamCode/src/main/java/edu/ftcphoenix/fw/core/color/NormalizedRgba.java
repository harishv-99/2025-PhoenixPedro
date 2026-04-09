package edu.ftcphoenix.fw.core.color;

/**
 * Immutable normalized RGBA color sample.
 *
 * <p>This is the normalized sibling of {@link Rgba}. FTC's
 * {@code NormalizedColorSensor} typically reports channels in a stable normalized range
 * (commonly around {@code 0..1}), often with configurable gain. Phoenix intentionally treats the
 * values as normalized-but-not-magical: ratio-based logic still tends to transfer better than raw
 * thresholding on a single channel.</p>
 *
 * <p>This value object stays intentionally parallel to {@link Rgba}: both expose channel ratios,
 * max/min/chroma, and simple HSV-style derived values. For classification, normalized thresholds
 * on ratios/alpha/chroma usually transfer better than raw thresholds do. Hue can still be useful
 * for debugging, but it becomes noisy when chroma is low.</p>
 */
public final class NormalizedRgba {

    /**
     * Red channel, typically in normalized units.
     */
    public final double r;

    /**
     * Green channel, typically in normalized units.
     */
    public final double g;

    /**
     * Blue channel, typically in normalized units.
     */
    public final double b;

    /**
     * Alpha / clear / brightness-like channel, typically in normalized units.
     */
    public final double a;

    /**
     * Construct a normalized RGBA sample.
     */
    public NormalizedRgba(double r, double g, double b, double a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    /**
     * Construct a normalized RGB sample with alpha/clear set to 0.
     */
    public static NormalizedRgba rgb(double r, double g, double b) {
        return new NormalizedRgba(r, g, b, 0.0);
    }

    /**
     * @return r + g + b.
     */
    public double sumRgb() {
        return r + g + b;
    }

    /**
     * @return the largest RGB channel.
     */
    public double maxChannel() {
        return ColorMath.maxChannel(r, g, b);
    }

    /**
     * @return the smallest RGB channel.
     */
    public double minChannel() {
        return ColorMath.minChannel(r, g, b);
    }

    /**
     * @return a simple RGB chroma estimate ({@code max(rgb) - min(rgb)}).
     */
    public double chroma() {
        return ColorMath.chroma(r, g, b);
    }

    /**
     * @return red ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double rRatio() {
        return ColorMath.ratio(r, sumRgb());
    }

    /**
     * @return green ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double gRatio() {
        return ColorMath.ratio(g, sumRgb());
    }

    /**
     * @return blue ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double bRatio() {
        return ColorMath.ratio(b, sumRgb());
    }

    /**
     * @return hue in degrees in [0,360), or 0 if chroma is 0.
     */
    public double hueDeg() {
        return ColorMath.hueDeg(r, g, b);
    }

    /**
     * @return HSV saturation in [0,1], or 0 when {@link #value()} is 0.
     */
    public double saturation() {
        return ColorMath.saturation(r, g, b);
    }

    /**
     * @return HSV value ({@code max(rgb)}).
     */
    public double value() {
        return ColorMath.value(r, g, b);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "NormalizedRgba(r=" + r + ", g=" + g + ", b=" + b + ", a=" + a + ")";
    }
}
