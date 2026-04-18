package edu.ftcphoenix.fw.spatial;

/**
 * Solved aim relationship from one spatial solve lane.
 *
 * <p>The stored {@link #aimErrorRad} is the signed rotation needed for the sampled aim frame's
 * +X axis to align with the requested aim target. Positive values are CCW / left turns.</p>
 */
public final class AimSolution {

    public final double aimErrorRad;
    public final double quality;
    public final double ageSec;

    public AimSolution(double aimErrorRad, double quality, double ageSec) {
        this.aimErrorRad = aimErrorRad;
        this.quality = quality;
        this.ageSec = ageSec;
    }

    @Override
    public String toString() {
        return "AimSolution{aimErrorRad=" + aimErrorRad + ", quality=" + quality + ", ageSec=" + ageSec + '}';
    }
}
