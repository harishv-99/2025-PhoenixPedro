package edu.ftcphoenix.fw.core.color;

/**
 * Immutable raw RGBA color sample.
 *
 * <p>This is a small, framework-owned value object used to represent a color sensor reading
 * without depending on FTC SDK types. The channel values are <b>not</b> assumed to be in any
 * particular range (some sensors report 0-255, others report larger values). Treat them as
 * <em>arbitrary units</em> and prefer ratio-based logic when comparing colors.</p>
 *
 * <p>Phoenix intentionally keeps this as a minimal value object:
 * it has a compact {@link #toString()} for logs and a few tiny helpers commonly used in
 * classification (sum and channel ratios).</p>
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
     * @return red ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double rRatio() {
        double s = sumRgb();
        return (s <= 0.0) ? 0.0 : ((double) r / s);
    }

    /**
     * @return green ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double gRatio() {
        double s = sumRgb();
        return (s <= 0.0) ? 0.0 : ((double) g / s);
    }

    /**
     * @return blue ratio in [0,1] relative to {@link #sumRgb()}, or 0 if sum is 0.
     */
    public double bRatio() {
        double s = sumRgb();
        return (s <= 0.0) ? 0.0 : ((double) b / s);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "Rgba(r=" + r + ", g=" + g + ", b=" + b + ", a=" + a + ")";
    }
}
