package edu.ftcphoenix.fw.core.color;

/**
 * Immutable raw RGBA color sample.
 *
 * <p>This is a small, framework-owned value object used to represent a color sensor reading
 * without depending on FTC SDK types. The channel values are <b>not</b> assumed to be in any
 * particular range (some sensors report 0-255, others report larger values). Treat them as
 * <em>arbitrary units</em> and prefer ratio-based logic when comparing colors.</p>
 *
 * <p>{@code Rgba} intentionally stays parallel to {@link NormalizedRgba} where that still makes
 * sense: both expose channel ratios, RGB max/min/chroma, and simple HSV-style derived values.
 * The important caveat is that raw thresholds on alpha/value/chroma are typically much less
 * portable across sensors, gain settings, and lighting than normalized thresholds are.</p>
 */
public final class Rgba {

    /**
     * Red channel, in sensor native units.
     */
    public final int r;

    /** Green channel, in sensor native units. */
    public final int g;

    /** Blue channel, in sensor native units. */
    public final int b;

    /** Alpha / clear channel (if available), in sensor native units. */
    public final int a;

    /**
     * Construct a raw RGBA sample.
     */
    public Rgba(int r, int g, int b, int a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    /**
     * Construct an RGB sample with alpha/clear set to 0.
     */
    public static Rgba rgb(int r, int g, int b) {
        return new Rgba(r, g, b, 0);
    }

    /** @return r + g + b as a double (avoids int overflow surprises). */
    public double sumRgb() {
        return (double) r + (double) g + (double) b;
    }

    /**
     * @return the largest RGB channel, in raw sensor units.
     */
    public double maxChannel() {
        return ColorMath.maxChannel(r, g, b);
    }

    /**
     * @return the smallest RGB channel, in raw sensor units.
     */
    public double minChannel() {
        return ColorMath.minChannel(r, g, b);
    }

    /**
     * @return a simple RGB chroma estimate ({@code max(rgb) - min(rgb)}) in raw sensor units.
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
     * @return HSV value ({@code max(rgb)}) in raw sensor units.
     */
    public double value() {
        return ColorMath.value(r, g, b);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "Rgba(r=" + r + ", g=" + g + ", b=" + b + ", a=" + a + ")";
    }
}
