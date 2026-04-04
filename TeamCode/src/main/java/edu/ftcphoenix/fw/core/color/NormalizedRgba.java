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
 * <p>This value object mirrors the small helper surface of {@link Rgba} and adds a simple
 * {@link #chroma()} helper that is often useful as a quick "how colorful is this sample?" signal
 * when deciding whether to trust a classification.</p>
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
        return Math.max(r, Math.max(g, b));
    }

    /**
     * @return the smallest RGB channel.
     */
    public double minChannel() {
        return Math.min(r, Math.min(g, b));
    }

    /**
     * @return a simple RGB chroma estimate ({@code max(rgb) - min(rgb)}).
     */
    public double chroma() {
        return maxChannel() - minChannel();
    }

    /**
     * @return red ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double rRatio() {
        double s = sumRgb();
        return (s <= 0.0) ? 0.0 : (r / s);
    }

    /**
     * @return green ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double gRatio() {
        double s = sumRgb();
        return (s <= 0.0) ? 0.0 : (g / s);
    }

    /**
     * @return blue ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double bRatio() {
        double s = sumRgb();
        return (s <= 0.0) ? 0.0 : (b / s);
    }

    @Override
    public String toString() {
        return "NormalizedRgba(r=" + r + ", g=" + g + ", b=" + b + ", a=" + a + ")";
    }
}
