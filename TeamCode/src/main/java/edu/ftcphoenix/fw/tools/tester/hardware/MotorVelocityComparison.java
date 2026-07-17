package edu.ftcphoenix.fw.tools.tester.hardware;

/**
 * Computes measurement-only motor velocity comparisons for {@link DcMotorPowerTester}.
 *
 * <p>This package-private diagnostic helper deliberately is not a framework feedback Source. It
 * compares direct and interval-derived encoder rates without selecting a production feedback
 * strategy. Position deltas use Java's signed 32-bit wraparound, which preserves the encoder delta
 * as long as fewer than {@code 2^31} ticks occur between accepted samples.</p>
 */
final class MotorVelocityComparison {

    /** One same-cycle snapshot of the values needed for the hardware comparison. */
    static final class Sample {
        final long cycle;
        final double timeSec;
        final int positionTicks;
        final long deltaPositionTicks;
        final double sampleIntervalSec;
        final double directVelocityTicksPerSec;
        final double derivedVelocityTicksPerSec;
        final double directMinusDerivedTicksPerSec;
        final boolean derivedVelocityAvailable;

        private Sample(long cycle,
                       double timeSec,
                       int positionTicks,
                       long deltaPositionTicks,
                       double sampleIntervalSec,
                       double directVelocityTicksPerSec,
                       double derivedVelocityTicksPerSec,
                       boolean derivedVelocityAvailable) {
            this.cycle = cycle;
            this.timeSec = timeSec;
            this.positionTicks = positionTicks;
            this.deltaPositionTicks = deltaPositionTicks;
            this.sampleIntervalSec = sampleIntervalSec;
            this.directVelocityTicksPerSec = directVelocityTicksPerSec;
            this.derivedVelocityTicksPerSec = derivedVelocityTicksPerSec;
            this.directMinusDerivedTicksPerSec = derivedVelocityAvailable
                    && Double.isFinite(directVelocityTicksPerSec)
                    ? directVelocityTicksPerSec - derivedVelocityTicksPerSec
                    : Double.NaN;
            this.derivedVelocityAvailable = derivedVelocityAvailable;
        }
    }

    private long sampledCycle = Long.MIN_VALUE;
    private Sample cachedSample;

    private boolean hasBaseline;
    private double baselineTimeSec;
    private int baselinePositionTicks;

    /** Clear the accepted position/time baseline and same-cycle cache. */
    void reset() {
        sampledCycle = Long.MIN_VALUE;
        cachedSample = null;
        hasBaseline = false;
        baselineTimeSec = Double.NaN;
        baselinePositionTicks = 0;
    }

    /**
     * Accept one motor snapshot for a loop cycle.
     *
     * <p>Repeated calls in the same cycle return the first snapshot. A first finite-time sample
     * establishes a baseline. Equal timestamps do not consume position movement, while a regressing
     * timestamp establishes a fresh baseline. A non-finite timestamp or derived result is reported
     * as unavailable without poisoning the last valid baseline.</p>
     */
    Sample sample(long cycle,
                  double timeSec,
                  int positionTicks,
                  double directVelocityTicksPerSec) {
        if (cachedSample != null && sampledCycle == cycle) {
            return cachedSample;
        }

        sampledCycle = cycle;

        long deltaPositionTicks = 0L;
        double sampleIntervalSec = Double.NaN;
        double derivedVelocityTicksPerSec = Double.NaN;
        boolean derivedVelocityAvailable = false;

        if (Double.isFinite(timeSec)) {
            if (!hasBaseline || timeSec < baselineTimeSec) {
                acceptBaseline(timeSec, positionTicks);
            } else if (timeSec > baselineTimeSec) {
                int rolloverAwareDelta = positionTicks - baselinePositionTicks;
                deltaPositionTicks = rolloverAwareDelta;
                sampleIntervalSec = timeSec - baselineTimeSec;

                double candidate = deltaPositionTicks / sampleIntervalSec;
                if (Double.isFinite(candidate)) {
                    derivedVelocityTicksPerSec = candidate;
                    derivedVelocityAvailable = true;
                    acceptBaseline(timeSec, positionTicks);
                }
            }
        }

        cachedSample = new Sample(
                cycle,
                timeSec,
                positionTicks,
                deltaPositionTicks,
                sampleIntervalSec,
                directVelocityTicksPerSec,
                derivedVelocityTicksPerSec,
                derivedVelocityAvailable);
        return cachedSample;
    }

    /** Replace the accepted finite-time position baseline. */
    private void acceptBaseline(double timeSec, int positionTicks) {
        hasBaseline = true;
        baselineTimeSec = timeSec;
        baselinePositionTicks = positionTicks;
    }
}
