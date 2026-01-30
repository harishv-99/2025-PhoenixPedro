package edu.ftcphoenix.fw.core.source;

import java.util.Objects;
import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;

import edu.ftcphoenix.fw.core.control.HysteresisBoolean;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Source} that produces a {@code double} each loop.
 *
 * <p>{@code ScalarSource} is the principled generalization of the old {@code Axis} abstraction:
 * "something that returns a double each loop". It is used for gamepad sticks/triggers,
 * sensor readings, and generated targets.</p>
 *
 * <p>All sampling is done through {@link #getAsDouble(LoopClock)} so sources can be stateful
 * (filters) and still respect Phoenix's "one loop, one heartbeat" design.</p>
 */
public interface ScalarSource extends Source<Double> {

    /**
     * Sample the current value.
     *
     * <p>Implementations may ignore {@code clock} (stateless sources) or use it for
     * dt-based filters / idempotence-by-cycle.</p>
     */
    double getAsDouble(LoopClock clock);

    @Override
    default Double get(LoopClock clock) {
        return getAsDouble(clock);
    }


    /**
     * Memoize this scalar for the current {@link LoopClock#cycle()}.
     *
     * <p>The returned source samples the upstream source at most once per cycle and returns the cached
     * value for any additional reads in that same cycle. Prefer using this for raw hardware reads
     * (distance sensors, encoders) or any derived value consumed by multiple subsystems.</p>
     */
    default ScalarSource memoized() {
        ScalarSource self = this;
        return new ScalarSource() {
            private long lastCycle = Long.MIN_VALUE;
            private double last = 0.0;

            @Override
            public double getAsDouble(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;
                last = self.getAsDouble(clock);
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                last = 0.0;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "memo" : prefix;
                dbg.addData(p + ".class", "MemoizedScalar");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Hold / stability helpers
    // ---------------------------------------------------------------------------------------------

    /**
     * Hold the last valid value for up to {@code maxHoldSec} seconds.
     *
     * <p>This is a scalar specialization of {@link Source#holdLastValid(java.util.function.Predicate, double, Object)}
     * that avoids boxing and provides a common "sensor validity" tool for automation.</p>
     *
     * <p>Implementation uses {@link LoopClock#nowSec()} so the max hold is enforced even if this
     * source is not sampled every loop.</p>
     *
     * @param isValid    predicate that defines which samples are valid
     * @param maxHoldSec maximum age of the held value in seconds; must be {@code >= 0}
     * @param fallback   value returned when no valid value is available (or the hold has expired)
     */
    default ScalarSource holdLastValid(DoublePredicate isValid, double maxHoldSec, double fallback) {
        Objects.requireNonNull(isValid, "isValid");
        if (maxHoldSec < 0.0) {
            throw new IllegalArgumentException("maxHoldSec must be >= 0, got " + maxHoldSec);
        }

        ScalarSource self = this;
        return new ScalarSource() {
            private long lastCycle = Long.MIN_VALUE;
            private double lastOut = fallback;

            private double lastValid = fallback;
            private boolean hasValid = false;
            private double lastValidSec = Double.NEGATIVE_INFINITY;
            /** The last time {@link #getAsDouble(LoopClock)} was sampled (for debug only). */
            private double lastSampleSec = Double.NEGATIVE_INFINITY;

            @Override
            public double getAsDouble(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return lastOut;
                }
                lastCycle = cyc;

                double cur = self.getAsDouble(clock);
                double now = clock.nowSec();
                lastSampleSec = now;

                if (isValid.test(cur)) {
                    hasValid = true;
                    lastValid = cur;
                    lastValidSec = now;
                    lastOut = cur;
                    return lastOut;
                }

                if (hasValid && (now - lastValidSec) <= maxHoldSec) {
                    lastOut = lastValid;
                    return lastOut;
                }

                lastOut = fallback;
                return lastOut;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                lastOut = fallback;
                hasValid = false;
                lastValid = fallback;
                lastValidSec = Double.NEGATIVE_INFINITY;
                lastSampleSec = Double.NEGATIVE_INFINITY;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "holdLastValid" : prefix;
                double ageSec = hasValid ? Math.max(0.0, lastSampleSec - lastValidSec) : Double.POSITIVE_INFINITY;
                dbg.addData(p + ".class", "HoldLastValidScalar")
                        .addData(p + ".maxHoldSec", maxHoldSec)
                        .addData(p + ".hasValid", hasValid)
                        .addData(p + ".lastValidAgeSec", ageSec)
                        .addData(p + ".fallback", fallback);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Hold the last finite value for up to {@code maxHoldSec} seconds.
     *
     * <p>This is a convenient way to deal with sources that may occasionally return NaN/Inf
     * (for example, a vision measurement that sometimes fails to solve).</p>
     */
    default ScalarSource holdLastFinite(double maxHoldSec, double fallback) {
        return holdLastValid(Double::isFinite, maxHoldSec, fallback);
    }

    // ---------------------------------------------------------------------------------------------
    // Common scalar transforms
    // ---------------------------------------------------------------------------------------------

    /**
     * Clamp this source into [{@code min}, {@code max}].
     */
    default ScalarSource clamped(double min, double max) {
        ScalarSource self = this;
        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                return MathUtil.clamp(self.getAsDouble(clock), min, max);
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "clamp" : prefix;
                dbg.addData(p + ".class", "ClampedScalar")
                        .addData(p + ".min", min)
                        .addData(p + ".max", max);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Scale this source by a constant factor.
     */
    default ScalarSource scaled(double factor) {
        ScalarSource self = this;
        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                return self.getAsDouble(clock) * factor;
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "scale" : prefix;
                dbg.addData(p + ".class", "ScaledScalar")
                        .addData(p + ".factor", factor);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Invert the sign of this source.
     */
    default ScalarSource inverted() {
        return scaled(-1.0);
    }

    /**
     * Apply a symmetric deadband around 0. Values within [-deadband, +deadband] become 0.
     */
    default ScalarSource deadband(double deadband) {
        ScalarSource self = this;
        double db = Math.abs(deadband);
        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                return MathUtil.deadband(self.getAsDouble(clock), db);
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "deadband" : prefix;
                dbg.addData(p + ".class", "DeadbandScalar")
                        .addData(p + ".deadband", db);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Apply a deadband centered at 0 and renormalize the remaining range back to [{@code min}, {@code max}].
     *
     * <p>This is useful when you want to remove small stick drift but still reach full scale at the extremes.</p>
     */
    default ScalarSource deadbandNormalized(double deadband, double min, double max) {
        ScalarSource self = this;
        final double db = Math.abs(deadband);

        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                double v = MathUtil.clamp(self.getAsDouble(clock), min, max);

                if (Math.abs(v) <= db) {
                    return 0.0;
                }

                // Positive side: [db..max] -> [0..max]
                if (v > 0.0) {
                    if (max <= db) {
                        return MathUtil.clamp(v, min, max);
                    }
                    double t = (v - db) / (max - db);
                    double out = t * max;
                    return MathUtil.clamp(out, min, max);
                }

                // Negative side: [min..-db] -> [min..0]
                if (min >= -db) {
                    return MathUtil.clamp(v, min, max);
                }
                double t = (v + db) / (min + db); // denominator negative
                double out = t * min;
                return MathUtil.clamp(out, min, max);
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "deadbandNorm" : prefix;
                dbg.addData(p + ".class", "DeadbandNormalizedScalar")
                        .addData(p + ".deadband", db)
                        .addData(p + ".min", min)
                        .addData(p + ".max", max);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Drive-style shaping: deadband -> normalize -> pow(expo) -> rescale.
     *
     * <p>This matches the behavior used historically by Phoenix drive stick shaping.</p>
     */
    default ScalarSource shaped(double deadband, double expo, double min, double max) {
        ScalarSource self = this;
        final double db = Math.abs(deadband);
        final double e = Math.max(1.0, expo);

        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                double v = MathUtil.clamp(self.getAsDouble(clock), min, max);

                if (Math.abs(v) <= db) {
                    return 0.0;
                }

                // Positive side shaping.
                if (v > 0.0) {
                    double sideMax = max;
                    if (sideMax <= db) {
                        return MathUtil.clamp(v, min, max);
                    }
                    double norm = (v - db) / (sideMax - db); // db -> 0, max -> 1
                    norm = MathUtil.clamp(norm, 0.0, 1.0);
                    double shaped = Math.pow(norm, e) * sideMax;
                    return MathUtil.clamp(shaped, min, max);
                }

                // Negative side shaping.
                double sideMag = -min; // magnitude of negative full-scale
                if (sideMag <= db) {
                    return MathUtil.clamp(v, min, max);
                }
                double norm = ((-v) - db) / (sideMag - db); // |v| in (db..sideMag] -> (0..1]
                norm = MathUtil.clamp(norm, 0.0, 1.0);
                double shapedMag = Math.pow(norm, e) * sideMag;
                double shaped = -shapedMag;
                return MathUtil.clamp(shaped, min, max);
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "shape" : prefix;
                dbg.addData(p + ".class", "ShapedScalar")
                        .addData(p + ".deadband", db)
                        .addData(p + ".expo", e)
                        .addData(p + ".min", min)
                        .addData(p + ".max", max);
                self.debugDump(dbg, p + ".src");
            }
        };
    }


    // ---------------------------------------------------------------------------------------------
    // Conversions & boolean helpers
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert to a boolean that is true when this value is strictly greater than {@code threshold}.
     */
    default BooleanSource above(double threshold) {
        ScalarSource self = this;
        return new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return self.getAsDouble(clock) > threshold;
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "above" : prefix;
                dbg.addData(p + ".class", "AboveBoolean")
                        .addData(p + ".threshold", threshold);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Convert to a boolean that is true when this value is strictly less than {@code threshold}.
     */
    default BooleanSource below(double threshold) {
        ScalarSource self = this;
        return new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return self.getAsDouble(clock) < threshold;
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "below" : prefix;
                dbg.addData(p + ".class", "BelowBoolean")
                        .addData(p + ".threshold", threshold);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Create a hysteresis boolean that turns ON when {@code value <= enterThreshold} and
     * turns OFF when {@code value >= exitThreshold}.
     *
     * <p>This is the common pattern for "idle" / "near zero" detection.</p>
     */
    default BooleanSource hysteresisBelow(double enterThreshold, double exitThreshold) {
        ScalarSource self = this;
        HysteresisBoolean latch = HysteresisBoolean.onWhenBelowOffWhenAbove(enterThreshold, exitThreshold);
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;
                last = latch.update(self.getAsDouble(clock));
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                latch.reset(false);
                lastCycle = Long.MIN_VALUE;
                last = false;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "hystBelow" : prefix;
                dbg.addData(p + ".class", "HysteresisBelowBoolean");
                latch.debugDump(dbg, p + ".latch");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Create a hysteresis boolean that turns ON when {@code value >= enterThreshold} and
     * turns OFF when {@code value <= exitThreshold}.
     *
     * <p>This is the common pattern for "valid when high" detection.</p>
     */
    default BooleanSource hysteresisAbove(double enterThreshold, double exitThreshold) {
        ScalarSource self = this;
        HysteresisBoolean latch = HysteresisBoolean.onWhenAboveOffWhenBelow(enterThreshold, exitThreshold);
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;
                last = latch.update(self.getAsDouble(clock));
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                latch.reset(false);
                lastCycle = Long.MIN_VALUE;
                last = false;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "hystAbove" : prefix;
                dbg.addData(p + ".class", "HysteresisAboveBoolean");
                latch.debugDump(dbg, p + ".latch");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Factories
    // ---------------------------------------------------------------------------------------------

    /**
     * Create a scalar source from a raw supplier.
     *
     * <p>The supplier is sampled each time {@link #getAsDouble(LoopClock)} is called.
     * The {@code clock} parameter is ignored.</p>
     */
    static ScalarSource of(DoubleSupplier raw) {
        Objects.requireNonNull(raw, "raw");
        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                return raw.getAsDouble();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "raw" : prefix;
                dbg.addData(p + ".class", "RawScalar");
            }
        };
    }

    /**
     * Create a constant scalar source.
     */
    static ScalarSource constant(double value) {
        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                return value;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "const" : prefix;
                dbg.addData(p + ".class", "ConstantScalar")
                        .addData(p + ".value", value);
            }
        };
    }

    /**
     * 2D magnitude: {@code hypot(x, y)}.
     *
     * <p>Null sources are treated as 0.</p>
     */
    static ScalarSource magnitude(ScalarSource x, ScalarSource y) {
        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                double xv = (x != null) ? x.getAsDouble(clock) : 0.0;
                double yv = (y != null) ? y.getAsDouble(clock) : 0.0;
                return Math.hypot(xv, yv);
            }

            @Override
            public void reset() {
                if (x != null) x.reset();
                if (y != null) y.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "mag" : prefix;
                dbg.addData(p + ".class", "MagnitudeScalar");
                if (x != null) x.debugDump(dbg, p + ".x");
                if (y != null) y.debugDump(dbg, p + ".y");
            }
        };
    }

    /**
     * Squared 2D magnitude: {@code x^2 + y^2}.
     *
     * <p>Null sources are treated as 0.</p>
     */
    static ScalarSource magnitudeSquared(ScalarSource x, ScalarSource y) {
        return new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                double xv = (x != null) ? x.getAsDouble(clock) : 0.0;
                double yv = (y != null) ? y.getAsDouble(clock) : 0.0;
                return xv * xv + yv * yv;
            }

            @Override
            public void reset() {
                if (x != null) x.reset();
                if (y != null) y.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "mag2" : prefix;
                dbg.addData(p + ".class", "MagnitudeSquaredScalar");
                if (x != null) x.debugDump(dbg, p + ".x");
                if (y != null) y.debugDump(dbg, p + ".y");
            }
        };
    }
}
